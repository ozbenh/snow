//! RBV (RAM-based video) of the Macintosh IIci and IIsi
//!
//! The RBV takes the place of VIA 2 with a few registers in the layout of a
//! VIA: there are no timers, no shift register and no data direction
//! registers, and only A4, A1 and A0 are decoded (IIci schematic). It
//! encodes the interrupt levels and sends the 60.15 Hz tick to VIA 1.
//!
//! Its video shows a frame buffer at the start of RAM bank A, which the MDU
//! reads for it, through the colour table of a Bt478. The monitor tells the
//! RBV its timings with its ID (Macintosh IIci Developer Note, chapter 5).
//! The frame is rendered at once at the start of the vertical blanking.

use crate::bus::Address;
use crate::bus::native_clock::C16M_FREQUENCY;
use crate::debuggable::Debuggable;
use crate::mac::MacMonitor;
use crate::renderer::{Renderer, render_packed};
use crate::tickable::Ticks;
use crate::types::Byte;

use anyhow::Result;
use proc_bitfield::bitfield;
use serde::{Deserialize, Serialize};

/// Monitors which can be connected
pub const MONITORS: &[MacMonitor] = &[
    MacMonitor::RGB12,
    MacMonitor::HiRes14,
    MacMonitor::PortraitBW,
];

bitfield! {
    /// Interrupt flag and enable registers
    #[derive(Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
    pub struct RegisterIRQ(pub u8): Debug, FromStorage, IntoStorage, DerefStorage {
        pub scsi_drq: bool @ 0,

        /// Any enabled slot interrupt
        pub any_slot: bool @ 1,

        pub scsi_irq: bool @ 3,

        pub asc: bool @ 4,

        /// Any enabled interrupt (read), set/clear (write)
        pub any: bool @ 7,
    }
}

bitfield! {
    /// Monitor parameters register
    #[derive(Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
    pub struct RegisterMonP(pub u8): Debug, FromStorage, IntoStorage, DerefStorage {
        /// Video depth: 2 to the power of this bits per pixel
        pub depth: u8 @ 0..=2,

        /// Monitor ID
        pub sense: u8 @ 3..=5,

        /// Video off
        pub video_off: bool @ 6,
    }
}

const REG_B: Address = 0x00;
const REG_EXP: Address = 0x01;
const REG_SLOT_IFR: Address = 0x02;
const REG_IFR: Address = 0x03;
const REG_MONP: Address = 0x10;
const REG_CHIP_TEST: Address = 0x11;
const REG_SLOT_IER: Address = 0x12;
const REG_IER: Address = 0x13;

/// Bits of the address which the RBV decodes
const ADDR_MASK: Address = 0x13;

/// Interrupt flags which can assert the interrupt output
const IRQ_MASK: u8 = 0x1F;

/// Slot interrupt flag of the built-in video
const SLOT_VBLANK: u8 = 1 << 6;

/// Bytes of the frame buffer the MDU reads in a burst for the RBV: eight
/// long words (Macintosh IIci Developer Note, chapter 5)
const FETCH_BYTES: Ticks = 32;

/// Clocks such a burst keeps bank A: five for the first long word and two
/// for each of the others, as the bursts of the CPU, and one more to end it
/// as after a write. It makes the RBV take 6, 13 and 26% of bank A at 1, 2
/// and 4 bits per pixel on a 13" monitor, and 13 and 26% at 1 and 2 on a
/// 15" Portrait, as on a IIci (Apple's figures); at 8 bits per pixel on a
/// 13", and 4 on a Portrait, it takes 51 and 52%, where Apple gives 64 and
/// 65%.
const FETCH_CLOCKS: Ticks = 20;

/// Bits of data register B which are inputs, NuBus TM1A and TM0A, here
/// always high
const B_INPUTS: u8 = 0x30;

/// Bt478 registers, selected by A4-A2
const DAC_WRITE_ADDRESS: Address = 0;
const DAC_PALETTE: Address = 1;
const DAC_PIXEL_MASK: Address = 2;
const DAC_READ_ADDRESS: Address = 3;

/// Bt478 colour table and DAC
#[derive(Serialize, Deserialize)]
struct Bt478 {
    /// Colour table (red, green, blue)
    palette: Vec<[u8; 3]>,

    /// Entry of the colour table which is read or written next
    address: u8,

    /// Component of the entry which is read or written next
    component: usize,

    /// Pixel read mask, ANDed with the pixel before the colour table
    pixel_mask: u8,

    /// Overlay registers and command register, stored only
    other: [u8; 4],
}

impl Bt478 {
    fn new() -> Self {
        Self {
            palette: vec![[0; 3]; 256],
            address: 0,
            component: 0,
            pixel_mask: 0xFF,
            other: [0; 4],
        }
    }

    fn next_component(&mut self) {
        self.component += 1;
        if self.component == 3 {
            self.component = 0;
            self.address = self.address.wrapping_add(1);
        }
    }

    fn read(&mut self, reg: Address) -> Byte {
        match reg {
            DAC_WRITE_ADDRESS | DAC_READ_ADDRESS => self.address,
            DAC_PALETTE => {
                let val = self.palette[usize::from(self.address)][self.component];
                self.next_component();
                val
            }
            DAC_PIXEL_MASK => self.pixel_mask,
            _ => self.other[reg as usize - 4],
        }
    }

    fn write(&mut self, reg: Address, val: Byte) {
        match reg {
            DAC_WRITE_ADDRESS | DAC_READ_ADDRESS => {
                self.address = val;
                self.component = 0;
            }
            DAC_PALETTE => {
                self.palette[usize::from(self.address)][self.component] = val;
                self.next_component();
            }
            DAC_PIXEL_MASK => self.pixel_mask = val,
            _ => self.other[reg as usize - 4] = val,
        }
    }
}

#[derive(Serialize, Deserialize)]
#[serde(bound = "")]
pub struct Rbv<TRenderer: Renderer> {
    #[serde(skip)]
    pub renderer: Option<TRenderer>,

    monitor: MacMonitor,
    dac: Bt478,

    /// Data register B, as written
    b: u8,
    /// Expansion register, stored only
    exp: u8,
    /// Chip test register, stored only
    chip_test: u8,
    monp: RegisterMonP,
    slot_ier: u8,
    ier: RegisterIRQ,

    /// Slot interrupt of the built-in video: the vertical blanking, unless
    /// cleared by writing the slot interrupt flag register. The ROM's video
    /// driver polls it with interrupts masked, for its start.
    vblank: bool,

    scsi_irq: bool,
    scsi_drq: bool,

    /// Level of the interrupt output of the ASC
    asc_line: bool,
    /// The ASC interrupt flag, which latches the assertion of the line
    asc_flag: bool,

    /// Time since the start of the vertical blanking, in cycles of C16M
    /// scaled by the bus frequency
    frame_clock: Ticks,

    /// Size and depth of the last frame shown, None if blanked, to log the
    /// mode changes
    #[serde(skip)]
    shown: Option<(usize, usize, u8)>,
}

impl<TRenderer> Rbv<TRenderer>
where
    TRenderer: Renderer,
{
    pub fn new(renderer: TRenderer, monitor: MacMonitor) -> Self {
        Self {
            renderer: Some(renderer),
            monitor,
            dac: Bt478::new(),
            b: 0,
            exp: 0,
            chip_test: 0,
            monp: RegisterMonP(0),
            slot_ier: 0,
            ier: RegisterIRQ(0),
            vblank: false,
            scsi_irq: false,
            scsi_drq: false,
            asc_line: false,
            asc_flag: false,
            frame_clock: 0,
            shown: None,
        }
    }

    /// RESET line. What it clears has not been verified on hardware, this
    /// disables the interrupts.
    pub fn reset(&mut self) {
        self.slot_ier = 0;
        self.ier = RegisterIRQ(0);
        self.vblank = false;
        self.asc_flag = false;
    }

    pub fn set_scsi(&mut self, irq: bool, drq: bool) {
        self.scsi_irq = irq;
        self.scsi_drq = drq;
    }

    /// The interrupt flag of the ASC latches the assertion of its line,
    /// even while it is disabled, and is cleared through the interrupt flag
    /// register, not when the line goes away (seen on a real IIci by Doug
    /// Brown, with an oscilloscope on the ASC interrupt output)
    pub fn set_asc_irq(&mut self, level: bool) {
        if level && !self.asc_line {
            self.asc_flag = true;
        }
        self.asc_line = level;
    }

    /// Slot interrupt flags, active low
    fn slot_ifr(&self) -> u8 {
        if self.vblank { !SLOT_VBLANK } else { 0xFF }
    }

    /// Interrupt flags, without the summary bit
    fn ifr(&self) -> RegisterIRQ {
        RegisterIRQ(0)
            .with_scsi_drq(self.scsi_drq)
            .with_any_slot(!self.slot_ifr() & self.slot_ier != 0)
            .with_scsi_irq(self.scsi_irq)
            .with_asc(self.asc_flag)
    }

    /// Level of the interrupt output
    pub fn irq(&self) -> bool {
        self.ifr().0 & self.ier.0 & IRQ_MASK != 0
    }

    /// Writes a register in which bit 7 selects setting or clearing of the
    /// other bits set in the written value
    fn write_set_clear(reg: u8, val: Byte) -> u8 {
        if val & 0x80 != 0 {
            reg | (val & 0x7F)
        } else {
            reg & !val
        }
    }

    pub fn read(&mut self, addr: Address) -> Byte {
        match addr & ADDR_MASK {
            REG_B => self.b | B_INPUTS,
            REG_EXP => self.exp,
            REG_SLOT_IFR => self.slot_ifr() & 0x7F,
            REG_IFR => self.ifr().with_any(self.irq()).0,
            REG_MONP => self.monp.with_sense(self.monitor.sense()[0]).0,
            REG_CHIP_TEST => self.chip_test,
            REG_SLOT_IER => self.slot_ier,
            // Bit 7 reads as set: the ROM checks this to tell the RBV apart
            REG_IER => self.ier.0 | 0x80,
            _ => unreachable!(),
        }
    }

    pub fn write(&mut self, addr: Address, val: Byte) {
        match addr & ADDR_MASK {
            REG_B => {
                if self.b & !val & 0x04 != 0 {
                    log::info!("Power off requested (not modelled)");
                }
                self.b = val;
            }
            REG_EXP => self.exp = val,
            REG_SLOT_IFR => {
                if val & SLOT_VBLANK != 0 {
                    self.vblank = false;
                }
            }
            REG_IFR => {
                if RegisterIRQ(val).asc() {
                    self.asc_flag = false;
                }
            }
            REG_MONP => {
                if RegisterMonP(val) != self.monp {
                    log::debug!("Monitor parameters register: {:02X}", val);
                }
                self.monp = RegisterMonP(val);
            }
            REG_CHIP_TEST => self.chip_test = val,
            REG_SLOT_IER => self.slot_ier = Self::write_set_clear(self.slot_ier, val),
            REG_IER => self.ier = RegisterIRQ(Self::write_set_clear(self.ier.0, val)),
            _ => unreachable!(),
        }
    }

    /// The Bt478 has its registers on A4-A2
    pub fn dac_read(&mut self, addr: Address) -> Byte {
        self.dac.read((addr >> 2) & 7)
    }

    pub fn dac_write(&mut self, addr: Address, val: Byte) {
        self.dac.write((addr >> 2) & 7, val);
    }

    pub fn monitor(&self) -> MacMonitor {
        self.monitor
    }

    /// Length of a frame and of its vertical blanking, in cycles of C16M,
    /// from the dot clock and the size of the frame (IIci and IIsi
    /// Developer Notes, chapter 4/5)
    fn frame_timing(&self) -> (Ticks, Ticks) {
        let (period, lines, blanking_lines) = match self.monitor {
            // 640x407 dots at 15.6672 MHz, 23 lines of blanking
            MacMonitor::RGB12 => (260_480, 407, 23),
            // 832x918 dots at 57.2832 MHz, 48 lines of blanking
            MacMonitor::PortraitBW => (208_896, 918, 48),
            // 864x525 dots at 30.24 MHz, 45 lines of blanking
            _ => (235_008, 525, 45),
        };
        (period, period * blanking_lines / lines)
    }

    /// Clocks a CPU access to bank A, `offset` clocks of the bus from now,
    /// waits for a burst of the RBV in progress (Developer Note, chapter 5).
    /// The RBV reads the frame buffer in bursts spread evenly over the lines
    /// of the picture, and none during the vertical blanking.
    pub fn fetch_hold(&self, offset: Ticks, bus_frequency: Ticks) -> Ticks {
        if self.monp.video_off() {
            return 0;
        }
        let (period, blanking) = self.frame_timing();
        let period = period * bus_frequency;
        let blanking = blanking * bus_frequency;
        let pos = (self.frame_clock + offset * C16M_FREQUENCY) % period;
        if pos < blanking {
            return 0;
        }
        let line = (period - blanking) / self.monitor.height() as Ticks;
        let line_bytes = (self.monitor.width() << self.monp.depth().min(3)) as Ticks / 8;
        let interval = line * FETCH_BYTES / line_bytes;
        let into = (pos - blanking) % interval;
        let fetch = FETCH_CLOCKS * C16M_FREQUENCY;
        if into < fetch {
            (fetch - into).div_ceil(C16M_FREQUENCY)
        } else {
            0
        }
    }

    /// Advances by a number of cycles of the bus. Returns true at the start
    /// of the vertical blanking.
    pub fn tick(&mut self, ticks: Ticks, bus_frequency: Ticks) -> bool {
        let (period, blanking) = self.frame_timing();
        self.frame_clock += ticks * C16M_FREQUENCY;
        if self.frame_clock >= period * bus_frequency {
            self.frame_clock -= period * bus_frequency;
            self.vblank = true;
            return true;
        }
        if self.frame_clock >= blanking * bus_frequency {
            self.vblank = false;
        }
        false
    }

    /// The RGBA pixel of each of the N pixel values. Below 8 bits per
    /// pixel, the RBV drives the upper pixel lines of the Bt478 high.
    fn palette_lut<const N: usize>(&self) -> [[u8; 4]; N] {
        let bpp = N.trailing_zeros();
        std::array::from_fn(|v| {
            let index = ((0xFF << bpp) as u8 | v as u8) & self.dac.pixel_mask;
            let rgb = self.dac.palette[usize::from(index)];
            // The monochrome monitors show the blue signal
            if self.monitor.has_color() {
                [rgb[0], rgb[1], rgb[2], 0xFF]
            } else {
                [rgb[2], rgb[2], rgb[2], 0xFF]
            }
        })
    }

    fn render_lines<const N: usize>(&self, buf: &mut [u8], fb: &[u8], width: usize) {
        let lut = self.palette_lut::<N>();
        let row_bytes = width * N.trailing_zeros() as usize / 8;
        for (line, row) in buf.chunks_exact_mut(width * 4).zip(fb.chunks(row_bytes)) {
            render_packed(row, line, &lut);
        }
    }

    /// Renders the frame from `fb`, RAM bank A
    pub fn render(&mut self, fb: &[u8]) -> Result<()> {
        if self.monp.video_off() {
            return self.blank();
        }
        let (width, height) = (self.monitor.width(), self.monitor.height());
        let depth = self.monp.depth().min(3);
        let shown = Some((width, height, depth));
        if shown != self.shown {
            log::debug!("Video mode: {}x{} at {} bpp", width, height, 1 << depth);
            self.shown = shown;
        }
        // We have to move the renderer so we don't upset the borrow checker
        let mut renderer = self.renderer.take().unwrap();
        let buf = renderer.buffer_mut();
        buf.set_size(width, height);
        match depth {
            0 => self.render_lines::<2>(buf, fb, width),
            1 => self.render_lines::<4>(buf, fb, width),
            2 => self.render_lines::<16>(buf, fb, width),
            _ => self.render_lines::<256>(buf, fb, width),
        }
        self.renderer = Some(renderer);
        self.renderer.as_mut().unwrap().update()
    }

    /// Shows a black screen
    pub fn blank(&mut self) -> Result<()> {
        if self.shown.take().is_some() {
            log::debug!("Video off");
        }
        let (width, height) = (self.monitor.width(), self.monitor.height());
        let renderer = self.renderer.as_mut().unwrap();
        let buf = renderer.buffer_mut();
        buf.set_size(width, height);
        for pixel in buf.chunks_exact_mut(4) {
            pixel.copy_from_slice(&[0, 0, 0, 0xFF]);
        }
        renderer.update()
    }
}

impl<TRenderer> Debuggable for Rbv<TRenderer>
where
    TRenderer: Renderer,
{
    fn get_debug_properties(&self) -> crate::debuggable::DebuggableProperties {
        use crate::dbgprop_byte_bin;
        use crate::debuggable::*;

        vec![
            dbgprop_byte_bin!("Data register B", self.b | B_INPUTS),
            dbgprop_byte_bin!("Slot interrupt flags", self.slot_ifr() & 0x7F),
            dbgprop_byte_bin!("Slot interrupt enable", self.slot_ier),
            dbgprop_byte_bin!("Interrupt flags", self.ifr().with_any(self.irq()).0),
            dbgprop_byte_bin!("Interrupt enable", self.ier.0),
            dbgprop_byte_bin!(
                "Monitor parameters",
                self.monp.with_sense(self.monitor.sense()[0]).0
            ),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::renderer::NullRenderer;

    fn rbv() -> Rbv<NullRenderer> {
        Rbv::new(NullRenderer::new(0, 0).unwrap(), MacMonitor::HiRes14)
    }

    #[test]
    fn mirrors() {
        let mut rbv = rbv();
        rbv.write(REG_MONP, 0x03);
        for addr in [0x10, 0x14, 0x18, 0x1C, 0x1F0, 0x1A10] {
            assert_eq!(rbv.read(addr) & 7, 3, "{:04X}", addr);
        }
    }

    /// The test the ROM runs to tell an RBV apart: the enables read back as
    /// set or cleared, with bit 7 set (bits 5 and 6 aren't looked at)
    #[test]
    fn ier_reads_bit_7_set() {
        let mut rbv = rbv();
        rbv.write(REG_IER, 0xFF);
        let mut bit = 1;
        while bit != 0x80 {
            rbv.write(REG_IER, bit);
            let expect = !(bit << 1).wrapping_sub(1) | 0x80;
            assert_eq!(rbv.read(REG_IER) & 0x9F, expect & 0x9F);
            bit <<= 1;
        }
    }

    #[test]
    fn asc_flag_latches_until_cleared() {
        let mut rbv = rbv();
        rbv.set_asc_irq(true);
        rbv.set_asc_irq(false);
        assert!(RegisterIRQ(rbv.read(REG_IFR)).asc());
        // Enabling it now still raises the interrupt
        rbv.write(REG_IER, 0x90);
        assert!(rbv.irq());
        rbv.write(REG_IFR, 0x90);
        assert!(!RegisterIRQ(rbv.read(REG_IFR)).asc());
        assert!(!rbv.irq());
    }

    #[test]
    fn vblank_slot_interrupt() {
        let mut rbv = rbv();
        rbv.write(REG_SLOT_IER, 0xFF);
        rbv.write(REG_IER, 0x82);
        let (period, blanking) = rbv.frame_timing();
        assert!(!rbv.tick(period - 1, C16M_FREQUENCY));
        assert!(rbv.tick(1, C16M_FREQUENCY));
        assert_eq!(rbv.read(REG_SLOT_IFR), 0x3F);
        assert!(rbv.irq());
        rbv.write(REG_SLOT_IFR, SLOT_VBLANK);
        assert_eq!(rbv.read(REG_SLOT_IFR), 0x7F);
        assert!(!rbv.irq());

        // Without the write, it lasts as long as the vertical blanking
        rbv.tick(period, C16M_FREQUENCY);
        rbv.tick(blanking - 1, C16M_FREQUENCY);
        assert_eq!(rbv.read(REG_SLOT_IFR), 0x3F);
        rbv.tick(1, C16M_FREQUENCY);
        assert_eq!(rbv.read(REG_SLOT_IFR), 0x7F);
    }

    /// Share of the time the RBV keeps bank A, at each depth
    #[test]
    fn fetch_share() {
        for (monitor, shares) in [
            (MacMonitor::HiRes14, [64, 128, 256, 512]),
            (MacMonitor::PortraitBW, [130, 261, 522, 0]),
        ] {
            let mut rbv = Rbv::new(NullRenderer::new(0, 0).unwrap(), monitor);
            let (period, _) = rbv.frame_timing();
            let bus_frequency = 25_000_000;
            let clocks = period * bus_frequency / C16M_FREQUENCY;
            for (depth, share) in shares.into_iter().enumerate() {
                if share == 0 {
                    continue;
                }
                rbv.write(REG_MONP, depth as Byte);
                let busy = (0..clocks)
                    .filter(|&t| rbv.fetch_hold(t, bus_frequency) > 0)
                    .count() as Ticks;
                // In thousandths
                assert_eq!(busy * 1000 / clocks, share, "{monitor:?} depth {depth}");
            }
        }
    }

    #[test]
    fn dac_palette() {
        let mut rbv = rbv();
        rbv.dac_write(DAC_WRITE_ADDRESS << 2, 0xFE);
        for val in [1, 2, 3, 4, 5, 6] {
            rbv.dac_write(DAC_PALETTE << 2, val);
        }
        assert_eq!(rbv.dac.palette[0xFE], [1, 2, 3]);
        assert_eq!(rbv.dac.palette[0xFF], [4, 5, 6]);
        // One bit per pixel shows entries $FE and $FF
        let lut = rbv.palette_lut::<2>();
        assert_eq!(lut, [[1, 2, 3, 0xFF], [4, 5, 6, 0xFF]]);
    }
}
