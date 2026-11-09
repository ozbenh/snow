//! Bus of the Macintosh IIci, around the MDU (memory decode unit)
//!
//! The MDU decodes the address space and runs the RAM, in two banks. The
//! RBV takes the place of VIA 2, sends the interrupts to the CPU, and shows
//! a frame buffer in bank A. The other I/O devices are those of the Mac II
//! family, at the same places.

use std::thread;
use std::time::{Duration, Instant};

use super::rbv::{self, Rbv};
use crate::bus::access::{Bus68020, BusAccess, BusResponse, bus_access_byte, bus_access_long};
use crate::bus::native_clock::{C16M_FREQUENCY, C16M_PER_VIA, DeviceCycle, NativeClock, WaitTicks};
use crate::bus::{
    Address, Bus, BusMember, InspectRun, InspectableBus, IrqSource, inspect_read_bytes,
};
use crate::debuggable::Debuggable;
use crate::emulator::comm::EmulatorSpeed;
use crate::emulator::{EmuContext, MouseMode};
use crate::keymap::KeyEvent;
use crate::mac::adb::{AdbEvent, AdbKeyboard, AdbMouse};
use crate::mac::rtc::Rtc;
use crate::mac::scc::{RTXC_FREQUENCY, Scc, SccCh};
use crate::mac::scsi::controller::ScsiController;
use crate::mac::sound::asc::Asc;
use crate::mac::swim::Swim;
use crate::mac::via::Via;
use crate::mac::via_common::{LineState, ViaLine};
use crate::mac::{AbsoluteMouse, MacModel, MacMonitor, WARM_START_SIGNATURE};
use crate::renderer::{
    AUDIO_BUFFER_SAMPLES, AUDIO_CHANNELS, AudioProvider, Renderer, null_audio_sink,
};
use crate::tickable::{Tickable, Ticks};
use crate::types::{Byte, LatchingEvent, MouseEvent};

use anyhow::Result;
use log::*;
use serde::{Deserialize, Serialize};

/// Frequency of the bus of the real hardware, without overclocking: the
/// 25 MHz of the 68030
const BUS_FREQUENCY: Ticks = 25_000_000;

/// Wait states of a RAM or ROM read: a random RAM read and a ROM read take
/// five clocks (Macintosh IIci Developer Note, appendix A)
const READ_WAITS: Ticks = 2;

/// Wait states of a RAM write, which takes four clocks
const RAM_WRITE_WAITS: Ticks = 1;

/// Wait states of a NuBus transaction, approximate
const NUBUS_WAITS: Ticks = 4;

/// Time the SCC needs between accesses (2.2 us), in cycles of the bus
const SCC_RECOVERY: Ticks = BUS_FREQUENCY * 22 / 10_000_000;

/// VIA clock cycles of a period of the 60.15 Hz tick the RBV sends to VIA 1
const TICK60_PERIOD: Ticks = 13024;

/// The I/O devices repeat every 256 KB in the I/O space
const IO_MASK: Address = 0x3_FFFF;

/// Sizes of a bank of RAM: four SIMMs of 256 KB, 1, 4 or 16 MB (Macintosh
/// IIci Developer Note, chapter 3)
const BANK_SIZES: [usize; 4] = [1 << 20, 4 << 20, 16 << 20, 64 << 20];

/// What answers in a 16 MB block of the address space
#[derive(Clone, Copy, PartialEq, Eq)]
enum Region {
    None,
    /// RAM, in banks A ($0000 0000) and B ($0400 0000)
    Ram,
    /// ROM, also in place of the RAM until the first access to the ROM
    Rom,
    /// I/O devices
    Io,
    /// Test software region / extension ROM
    ExtRom,
    /// NuBus super slots
    SuperSlot,
}

const fn region(overlay: bool, block: usize) -> Region {
    match block {
        0x00..=0x3F if overlay => Region::Rom,
        0x00..=0x07 => Region::Ram,
        0x40..=0x4F => Region::Rom,
        0x50 => Region::Io,
        0x58..=0x5F => Region::ExtRom,
        0x60..=0xEF => Region::SuperSlot,
        _ => Region::None,
    }
}

/// Region of each 16 MB block, with the boot overlay off and on
const MAP: [[Region; 256]; 2] = {
    let mut map = [[Region::None; 256]; 2];
    let mut block = 0;
    while block < 256 {
        map[0][block] = region(false, block);
        map[1][block] = region(true, block);
        block += 1;
    }
    map
};

/// A bank of RAM: each has a 64 MB window, which repeats what it holds, as
/// the ROM finds out its size from that
#[derive(Clone, Copy, Serialize, Deserialize)]
struct Bank {
    /// Index of its first byte in the RAM
    base: usize,
    size: usize,
}

/// Splits `ram_size` into the sizes of banks A and B, with the smaller
/// SIMMs in bank A: "For best performance with on-board video, put the
/// smaller SIMMs in bank A" (Macintosh IIci Developer Note, figure 3-1)
fn bank_sizes(ram_size: usize) -> (usize, usize) {
    for a in BANK_SIZES {
        if a == ram_size {
            return (a, 0);
        }
        for b in BANK_SIZES {
            if b >= a && a + b == ram_size {
                return (a, b);
            }
        }
    }
    panic!("Unsupported RAM size: {}", ram_size)
}

#[derive(Serialize, Deserialize)]
#[serde(bound = "")]
pub struct MduBus<TRenderer: Renderer> {
    /// The currently emulated Macintosh model
    model: MacModel,

    rom: Vec<u8>,
    extension_rom: Vec<u8>,
    /// Bank A, then bank B
    pub(crate) ram: Vec<u8>,
    banks: [Bank; 2],

    pub(crate) via1: Via,
    pub(crate) rbv: Rbv<TRenderer>,
    pub(crate) scc: Scc,
    pub(crate) asc: Asc,
    /// Native time the SCC can be accessed again
    scc_ready: Ticks,
    asc_clock: Ticks,
    mouse_ready: bool,
    pub(crate) swim: Swim,
    pub(crate) scsi: ScsiController,

    rom_mask: usize,

    overlay: bool,

    /// Emulation speed setting
    pub(crate) speed: EmulatorSpeed,

    /// Counts clock ticks at native frequency, regardless of overclocking
    native_clock: NativeClock,

    /// VIA clock cycles into the period of the 60.15 Hz tick
    tick60_clock: Ticks,

    /// Last vblank time (for syncing to video)
    /// Not serializing this because it is only used for determining how long to
    /// sleep for in Video speed mode.
    #[serde(skip, default = "Instant::now")]
    vblank_time: Instant,
    vblank_clock: Ticks,

    /// Programmer's key pressed
    progkey_pressed: LatchingEvent,

    /// Mouse mode
    mouse_mode: MouseMode,
}

impl<TRenderer> MduBus<TRenderer>
where
    TRenderer: Renderer,
{
    /// Value to return on open bus
    const OPENBUS: u8 = 0;

    pub fn new(
        model: MacModel,
        rom: &[u8],
        extension_rom: Option<&[u8]>,
        renderer: TRenderer,
        monitor: MacMonitor,
        mouse_mode: MouseMode,
        ram_size: Option<usize>,
    ) -> Self {
        let ram_size = ram_size.unwrap_or_else(|| model.ram_size_default());
        let (size_a, size_b) = bank_sizes(ram_size);
        let monitor = if rbv::MONITORS.contains(&monitor) {
            monitor
        } else {
            MacMonitor::default()
        };

        if extension_rom.is_some() {
            log::info!("Extension ROM present");
        }

        let mut bus = Self {
            model,

            rom: Vec::from(rom),
            extension_rom: extension_rom.map(Vec::from).unwrap_or_default(),
            ram: vec![0; ram_size],
            banks: [
                Bank {
                    base: 0,
                    size: size_a,
                },
                Bank {
                    base: size_a,
                    size: size_b,
                },
            ],
            via1: Via::new(model),
            rbv: Rbv::new(renderer, monitor),
            scc_ready: 0,
            scc: Scc::new(RTXC_FREQUENCY),
            swim: Swim::new(model.fdd_drives(), model.fdd_hd(), C16M_FREQUENCY),
            scsi: ScsiController::new(),
            asc: Asc::default(),
            asc_clock: 0,
            mouse_ready: false,

            rom_mask: rom.len() - 1,

            overlay: true,
            speed: EmulatorSpeed::Accurate,
            native_clock: NativeClock::new(BUS_FREQUENCY),
            tick60_clock: 0,
            vblank_time: Instant::now(),
            vblank_clock: 0,
            progkey_pressed: LatchingEvent::default(),

            mouse_mode,
        };

        bus.disable_memtest();

        // Initialize ADB devices
        bus.via1.adb.add_device(AdbMouse::new());
        bus.via1.adb.add_device(AdbKeyboard::new());

        bus
    }

    /// Reinstalls things that can't be serialized and does some updates upon deserialization
    pub fn after_deserialize(&mut self, renderer: TRenderer) {
        self.rbv.renderer = Some(renderer);
        self.render().unwrap();

        self.asc.after_deserialize();
    }

    pub fn model(&self) -> MacModel {
        self.model
    }

    pub fn get_effective_speed(&self) -> f64 {
        self.via1.rtc.effective_speed()
    }

    pub fn set_audio_provider(&mut self, provider: &mut dyn AudioProvider) -> Result<()> {
        let sink = provider
            .create_stream(22050, AUDIO_CHANNELS as u8, AUDIO_BUFFER_SAMPLES as u16)
            .unwrap_or_else(|_| null_audio_sink());
        self.asc.set_sink(sink);

        self.scsi.bus_mut().set_audio_provider(provider)
    }

    /// Makes the ROM skip its memory test
    ///
    /// The ROM sizes bank B, then bank A, and builds its table of the RAM
    /// at the top of the first bank it finds, below which it looks for the
    /// warm start signature: 32 bytes below the top with two banks, 24
    /// with one.
    fn disable_memtest(&mut self) {
        let [a, b] = self.banks;
        let index = if b.size > 0 {
            b.base + b.size - 0x20
        } else {
            a.size - 0x18
        };
        info!("Skipping memory test");
        self.ram[index..index + 4].copy_from_slice(&WARM_START_SIGNATURE.1.to_be_bytes());
    }

    /// Index in the RAM of the byte at `addr` in the RAM space, if the bank
    /// has RAM
    #[inline(always)]
    fn ram_index(&self, addr: Address) -> Option<usize> {
        let bank = self.banks[(addr >> 26) as usize & 1];
        (bank.size > 0).then(|| bank.base + (addr as usize & (bank.size - 1)))
    }

    /// Wraps a device read result in a mapped region: logs a warning and falls
    /// back to open bus when the device did not handle the address. The
    /// enclosing region is considered mapped, so this never triggers a bus
    /// error.
    fn dev_read(addr: Address, val: Option<Byte>) -> Byte {
        val.unwrap_or_else(|| {
            warn!("Read from unimplemented address: {:08X}", addr);
            Self::OPENBUS
        })
    }

    /// Wraps a device write result in a mapped region: logs a warning if the
    /// device did not handle the address. The enclosing region is considered
    /// mapped, so this never triggers a bus error.
    fn dev_write(addr: Address, val: Byte, written: Option<()>) -> Option<()> {
        if written.is_none() {
            warn!("Write to unimplemented address: {:08X} {:02X}", addr, val);
        }
        Some(())
    }

    // RAM and ROM transfers stay within an aligned long word, which is in
    // one bank

    fn ram_read(&mut self, addr: Address, len: u32) -> Option<u32> {
        // An empty bank floats
        let Some(idx) = self.ram_index(addr) else {
            return Some(0);
        };
        let bytes = &self.ram[idx..idx + len as usize];
        Some(bytes.iter().fold(0, |v, &b| (v << 8) | u32::from(b)))
    }

    fn ram_write(&mut self, addr: Address, len: u32, val: u32) -> Option<()> {
        if let Some(idx) = self.ram_index(addr) {
            self.ram[idx..idx + len as usize]
                .copy_from_slice(&val.to_be_bytes()[4 - len as usize..]);
        }
        Some(())
    }

    fn rom_read(&mut self, addr: Address, len: u32) -> Option<u32> {
        let idx = addr as usize & self.rom_mask;
        if let Some(bytes) = self.rom.get(idx..idx + len as usize) {
            return Some(bytes.iter().fold(0, |v, &b| (v << 8) | u32::from(b)));
        }
        Some((addr..addr + len).fold(0, |v, a| {
            let idx = a as usize & self.rom_mask;
            (v << 8) | u32::from(self.rom.get(idx).copied().unwrap_or(Self::OPENBUS))
        }))
    }

    fn ext_rom_read(&mut self, addr: Address, len: u32) -> Option<u32> {
        Some((addr..addr + len).fold(0, |v, a| {
            let rom = self.extension_rom.get((a - 0x5800_0000) as usize);
            (v << 8) | u32::from(rom.copied().unwrap_or(Self::OPENBUS))
        }))
    }

    fn ignore_write(&mut self, _addr: Address, _len: u32, _val: u32) -> Option<()> {
        Some(())
    }

    fn no_write(&mut self, _addr: Address, _len: u32, _val: u32) -> Option<()> {
        None
    }

    fn open_read(&mut self, _addr: Address, _len: u32) -> Option<u32> {
        Some(Self::OPENBUS.into())
    }

    /// Unimplemented but some stray reads/writes end up here
    fn superslot_read(&mut self, addr: Address, _len: u32) -> Option<u32> {
        log::warn!("Unimplemented read from NuBus super slot: {:08X}", addr);
        // Open bus on every byte
        Some(u32::MAX)
    }

    fn superslot_write(&mut self, addr: Address, _len: u32, val: u32) -> Option<()> {
        log::warn!(
            "Unimplemented write to NuBus super slot: {:08X} {:08X}",
            addr,
            val
        );
        Some(())
    }

    fn via1_read(&mut self, addr: Address, _len: u32) -> Option<u32> {
        Some(Self::dev_read(addr, self.via1.read(addr)).into())
    }

    fn via1_write(&mut self, addr: Address, _len: u32, val: u32) -> Option<()> {
        Self::dev_write(addr, val as Byte, self.via1.write(addr, val as Byte));
        self.swim.sel = self.via1.pa().sel();
        Some(())
    }

    fn rbv_read(&mut self, addr: Address, _len: u32) -> Option<u32> {
        Some(self.rbv.read(addr).into())
    }

    fn rbv_write(&mut self, addr: Address, _len: u32, val: u32) -> Option<()> {
        self.rbv.write(addr, val as Byte);
        Some(())
    }

    fn dac_read(&mut self, addr: Address, _len: u32) -> Option<u32> {
        Some(self.rbv.dac_read(addr).into())
    }

    fn dac_write(&mut self, addr: Address, _len: u32, val: u32) -> Option<()> {
        self.rbv.dac_write(addr, val as Byte);
        Some(())
    }

    fn scc_read(&mut self, addr: Address, _len: u32) -> Option<u32> {
        Some(Self::dev_read(addr, self.scc.read(addr >> 1)).into())
    }

    fn scc_write(&mut self, addr: Address, _len: u32, val: u32) -> Option<()> {
        Self::dev_write(addr, val as Byte, self.scc.write(addr >> 1, val as Byte))
    }

    fn scsi_read(&mut self, addr: Address, _len: u32) -> Option<u32> {
        Some(Self::dev_read(addr, self.scsi.read(addr)).into())
    }

    fn scsi_write(&mut self, addr: Address, _len: u32, val: u32) -> Option<()> {
        Self::dev_write(addr, val as Byte, self.scsi.write(addr, val as Byte))
    }

    fn scsi_dma_read(&mut self, _addr: Address, _len: u32) -> Option<u32> {
        Some(self.scsi.read_dma().into())
    }

    fn scsi_dma_write(&mut self, _addr: Address, _len: u32, val: u32) -> Option<()> {
        self.scsi.write_dma(val as Byte);
        Some(())
    }

    fn asc_read(&mut self, addr: Address, _len: u32) -> Option<u32> {
        Some(Self::dev_read(addr, self.asc.read(addr & 0xFFF)).into())
    }

    fn asc_write(&mut self, addr: Address, _len: u32, val: u32) -> Option<()> {
        Self::dev_write(addr, val as Byte, self.asc.write(addr & 0xFFF, val as Byte))
    }

    fn swim_read(&mut self, addr: Address, _len: u32) -> Option<u32> {
        Some(Self::dev_read(addr, self.swim.read(addr)).into())
    }

    fn swim_write(&mut self, addr: Address, _len: u32, val: u32) -> Option<()> {
        Self::dev_write(addr, val as Byte, self.swim.write(addr, val as Byte))
    }

    /// The I/O devices are 8-bit ports on D31-D24 (IIci schematic), every
    /// 8 KB of the block, which repeats every 256 KB. Their cycles take
    /// native time: the VIA is synchronous with the VIA clock, the other
    /// devices start on a native tick, with the wait states of the Mac II
    /// family's GLUE for now.
    ///
    /// Nothing else answers in the block. What the MDU does there is not
    /// known: it reads as open bus here, without a bus error, as the ROM
    /// finds out which decoder the machine has by checking that VIA 1
    /// repeats 256 KB above itself but not 128 KB.
    fn io_cycle(&mut self, acc: BusAccess<Bus68020>) -> BusResponse {
        let device = (acc.addr & IO_MASK) >> 13;
        if device == 0 {
            return self.via_cycle(acc);
        }
        let w = match self.native_clock.device_sync(&acc) {
            DeviceCycle::Stall(n) => return BusResponse::Stall(n),
            DeviceCycle::Access(w) => w,
        };
        match device {
            2 => self.scc_cycle(acc, w),
            // SCSI pseudo-DMA only takes the lower 4KB of its slot
            3 if acc.addr & 0x1000 == 0 => {
                bus_access_byte(acc, w, self, Self::scsi_dma_read, Self::scsi_dma_write)
            }
            8 => {
                let w = self.native_clock.scale_dev_waits(w, 1);
                bus_access_byte(acc, w, self, Self::scsi_read, Self::scsi_write)
            }
            9 => bus_access_byte(acc, w, self, Self::scsi_dma_read, Self::scsi_dma_write),
            10 => {
                let w = self
                    .native_clock
                    .scale_dev_waits(w, if acc.write { 1 } else { 2 });
                bus_access_byte(acc, w, self, Self::asc_read, Self::asc_write)
            }
            11 => {
                let hold = self.swim.hold(&self.native_clock);
                let w = self.native_clock.scale_dev_waits(w, 1 + hold);
                bus_access_byte(acc, w, self, Self::swim_read, Self::swim_write)
            }
            // Bt478
            18 => bus_access_byte(acc, w, self, Self::dac_read, Self::dac_write),
            19 => bus_access_byte(acc, w, self, Self::rbv_read, Self::rbv_write),
            _ => bus_access_byte(acc, w, self, Self::open_read, Self::ignore_write),
        }
    }

    /// Accesses to the SCC are held off until it has had the time it needs
    /// since the previous one
    fn scc_cycle(&mut self, acc: BusAccess<Bus68020>, w: WaitTicks) -> BusResponse {
        // When overclocking, acc.pending is 0 after device_sync(), and native
        // ticks are CPU ticks otherwise
        let now = self.native_clock.native_ticks() + acc.pending;
        let hold = self.scc_ready.saturating_sub(now);
        // The access itself takes 3 clocks
        self.scc_ready = now + hold + 3 + SCC_RECOVERY;
        let w = self.native_clock.scale_dev_waits(w, hold);
        bus_access_byte(acc, w, self, Self::scc_read, Self::scc_write)
    }

    /// VIA accesses are synchronous with the VIA clock: one starts on a VIA
    /// tick, and the CPU is released half way through the VIA cycle (as seen
    /// with BusTiming on a real IIci)
    fn via_cycle(&mut self, acc: BusAccess<Bus68020>) -> BusResponse {
        match self.native_clock.via_sync(&acc, C16M_PER_VIA / 2) {
            DeviceCycle::Stall(n) => BusResponse::Stall(n),
            DeviceCycle::Access(w) => {
                bus_access_byte(acc, w, self, Self::via1_read, Self::via1_write)
            }
        }
    }

    /// Renders the frame buffer, at the start of bank A
    fn render(&mut self) -> Result<()> {
        let fb = &self.ram[..self.banks[0].size];
        self.rbv.render(fb)
    }

    /// Updates the mouse position (relative coordinates) and button state
    pub fn mouse_update_rel(&mut self, relx: i16, rely: i16, button: Option<bool>) {
        if self.mouse_mode == MouseMode::Disabled {
            return;
        }

        if button.is_some() {
            self.via1.adb.event(&AdbEvent::Mouse(MouseEvent {
                button,
                rel_movement: None,
            }));
        }

        if relx == 0 && rely == 0 {
            return;
        }

        match self.mouse_mode {
            MouseMode::Absolute => {
                // Handled through mouse_update_abs()
            }
            MouseMode::RelativeHw => {
                self.via1.adb.event(&AdbEvent::Mouse(MouseEvent {
                    button: None,
                    rel_movement: Some((relx.into(), rely.into())),
                }));
            }
            MouseMode::Disabled => unreachable!(),
        }
    }

    /// Sets the mouse emulation mode
    pub fn set_mouse_mode(&mut self, mode: MouseMode) {
        self.mouse_mode = mode;
    }

    /// Updates the mouse position in absolute mode, given the current value
    /// of RawMouse. Returns what the low memory globals of the OS should hold
    /// for it, if they should change.
    pub fn mouse_update_abs(
        &mut self,
        x: u16,
        y: u16,
        rawmouse: (u16, u16),
    ) -> Option<AbsoluteMouse> {
        if self.mouse_mode != MouseMode::Absolute {
            return None;
        }

        if !self.mouse_ready && rawmouse != (15, 15) {
            // Wait until the boot process has initialized the mouse position so we don't
            // interfere with the memory test.
            return None;
        }
        self.mouse_ready = true;

        // Report updated mouse coordinates to OS. The OS needs to see even a
        // small difference between the current (RawMouse) and new (MTemp)
        // position, otherwise the change is ignored.
        Some(AbsoluteMouse {
            mtemp: (x, y),
            rawmouse: (x.wrapping_add_signed(-1), y.wrapping_add_signed(1)),
        })
    }

    /// Configures emulator speed
    pub fn set_speed(&mut self, speed: EmulatorSpeed) {
        info!("Emulation speed: {:?}", speed);
        self.speed = speed;
    }

    /// Frequency of the bus of the real hardware, without overclocking
    #[inline(always)]
    pub const fn bus_frequency(&self) -> Ticks {
        BUS_FREQUENCY
    }

    /// Sets the frequency of the bus, or goes back to the one of the real
    /// hardware
    pub fn set_overclock(&mut self, bus_frequency: Option<u64>) {
        self.native_clock
            .set_cpu_frequency(bus_frequency.unwrap_or_else(|| self.bus_frequency()));
    }

    /// Programmer's key pressed
    pub fn progkey(&mut self) {
        self.progkey_pressed.set();
    }

    pub fn video_blank(&mut self) -> Result<()> {
        self.rbv.blank()
    }

    /// Dispatches a key event to the keyboard
    pub fn keyboard_event(&mut self, ke: KeyEvent) {
        self.via1.adb.event(&AdbEvent::Key(ke));
    }

    /// Releases all pressed inputs
    pub fn input_release_all(&mut self) {
        self.via1.adb.event(&AdbEvent::ReleaseAll);
    }

    pub fn rtc_mut(&mut self) -> &mut Rtc {
        &mut self.via1.rtc
    }
}

impl<TRenderer> Bus<Address, Byte> for MduBus<TRenderer>
where
    TRenderer: Renderer,
{
    type Cpu = Bus68020;

    fn get_mask(&self) -> Address {
        0xFFFFFFFF
    }

    #[inline(always)]
    fn access(&mut self, acc: BusAccess<Bus68020>) -> Result<BusResponse> {
        let block = (acc.addr >> 24) as usize;
        // The normal map takes over at the first access to the ROM
        // (Macintosh IIci Developer Note, chapter 2)
        if self.overlay && (0x40..=0x4F).contains(&block) {
            debug!("Overlay off");
            self.overlay = false;
        }

        let resp = match MAP[usize::from(self.overlay)][block] {
            // Everything but RAM and ROM sees the devices up to date, as they are ticked
            Region::SuperSlot if acc.pending > 0 => BusResponse::Stall(0),
            Region::Ram => {
                let waits = if acc.write {
                    RAM_WRITE_WAITS
                } else {
                    READ_WAITS
                };
                bus_access_long(acc, waits, self, Self::ram_read, Self::ram_write)
            }
            Region::Rom => {
                bus_access_long(acc, READ_WAITS, self, Self::rom_read, Self::ignore_write)
            }
            Region::Io => self.io_cycle(acc),
            Region::ExtRom => {
                bus_access_long(acc, READ_WAITS, self, Self::ext_rom_read, Self::no_write)
            }
            Region::SuperSlot => bus_access_long(
                acc,
                NUBUS_WAITS,
                self,
                Self::superslot_read,
                Self::superslot_write,
            ),
            Region::None => BusResponse::BusError,
        };

        if resp == BusResponse::BusError {
            let rw = if acc.write { "write" } else { "read" };
            warn!("Bus error ({}) at unmapped address: {:08X}", rw, acc.addr);
        }
        Ok(resp)
    }

    fn reset(&mut self, hard: bool) -> Result<bool> {
        if hard {
            // Clear RAM
            self.ram.fill(0);
            self.disable_memtest();
        }

        // Keep the RTC and ADB for PRAM and event channels
        let Via { adb, rtc, .. } = std::mem::replace(&mut self.via1, Via::new(self.model));
        self.via1.adb = adb;
        self.via1.rtc = rtc;

        self.swim.reset();
        self.scc = Scc::new(RTXC_FREQUENCY);
        self.rbv.reset();
        self.asc.reset();

        self.mouse_ready = false;
        self.overlay = true;
        self.swim.sel = self.via1.pa().sel();
        Ok(false)
    }

    fn localtalk_poll_needed(&self) -> bool {
        self.scc.localtalk_poll_needed(SccCh::B)
    }
}

impl<TRenderer> Tickable for MduBus<TRenderer>
where
    TRenderer: Renderer,
{
    fn tick(&mut self, ticks: Ticks, _: ()) -> Result<Ticks> {
        struct BusEmuContext {
            speed: EmulatorSpeed,
            bus_frequency: Ticks,
        }

        impl EmuContext for BusEmuContext {
            fn speed(&self) -> EmulatorSpeed {
                self.speed
            }

            fn bus_frequency(&self) -> Ticks {
                self.bus_frequency
            }
        }

        let ctx = &BusEmuContext {
            speed: self.speed,
            bus_frequency: self.bus_frequency(),
        };

        // The devices run at the native frequency, whatever the CPU is
        // overclocked to
        let mut native_ticks = 0;
        let mut cpu = ticks;
        while cpu > 0 {
            let step = self.native_clock.advance(cpu, None);
            if step.native == 0 && !step.via_tick {
                break;
            }
            cpu -= step.cpu_used;
            native_ticks += step.native;

            // The VIA clock is C16M / 20 (783.36 kHz)
            if step.via_tick {
                self.via1.tick(1, ())?;

                // 60.15 Hz interrupt, generated by the RBV
                self.tick60_clock += 1;
                if self.tick60_clock >= TICK60_PERIOD {
                    self.tick60_clock = 0;
                    self.via1.set_line(ViaLine::CA1, LineState::Pulse);
                }
            }
        }

        // Host frame pacing
        self.vblank_clock += native_ticks;
        while self.vblank_clock >= ctx.bus_frequency / 60 {
            self.vblank_clock -= ctx.bus_frequency / 60;

            let max_speed = match self.speed {
                // Sync to 60 fps video
                EmulatorSpeed::Video => Some(1.0),
                // Cap to the requested speedup multiplier
                EmulatorSpeed::FastForward(max_speed) => Some(max_speed),
                _ => None,
            };
            if let Some(max_speed) = max_speed {
                // Sleep until the time at which this frame is due, counted from
                // the time the last one was due so that the error of the sleep
                // does not add up.
                let due = self.vblank_time + Duration::from_secs_f64(1.0 / (60.0 * max_speed));
                let now = Instant::now();
                if now < due {
                    thread::sleep(due - now);
                    self.vblank_time = due;
                } else {
                    self.vblank_time = now;
                }
            }
        }

        // Video
        if self.rbv.tick(native_ticks, ctx.bus_frequency)
            && self.rbv.renderer.as_ref().unwrap().frame_wanted()
        {
            self.render()?;
        }

        // Audio
        self.asc_clock += native_ticks * C16M_FREQUENCY;
        while self.asc_clock >= self.asc.sample_period() * ctx.bus_frequency {
            self.asc_clock -= self.asc.sample_period() * ctx.bus_frequency;

            self.asc.tick(self.speed == EmulatorSpeed::Accurate)?;
        }
        self.rbv.set_asc_irq(self.asc.get_irq());

        self.scsi.tick(native_ticks, ctx)?;
        self.rbv.set_scsi(self.scsi.get_irq(), self.scsi.get_drq());

        self.swim.tick(native_ticks, ())?;
        self.scc.tick(native_ticks, ctx.bus_frequency);

        Ok(ticks)
    }
}

impl<TRenderer> IrqSource for MduBus<TRenderer>
where
    TRenderer: Renderer,
{
    /// The RBV encodes the interrupt levels (IIci schematic), as the Mac II
    /// family has them
    fn get_irq(&mut self) -> Option<u8> {
        if self.progkey_pressed.get_clear() {
            return Some(7);
        }
        if self.scc.get_irq() {
            return Some(4);
        }
        if self.rbv.irq() {
            return Some(2);
        }
        if self.via1.irq() {
            return Some(1);
        }
        None
    }
}

impl<TRenderer> InspectableBus<Address, Byte> for MduBus<TRenderer>
where
    TRenderer: Renderer,
{
    fn inspect_read(&mut self, addr: Address) -> Option<Byte> {
        let val = match MAP[usize::from(self.overlay)][(addr >> 24) as usize] {
            Region::Ram => self.ram_read(addr, 1),
            Region::Rom => self.rom_read(addr, 1),
            _ => None,
        };
        val.map(|v| v as Byte)
    }

    fn inspect_write(&mut self, addr: Address, val: Byte) -> Option<()> {
        match MAP[usize::from(self.overlay)][(addr >> 24) as usize] {
            Region::Ram => self.ram_write(addr, 1, val.into()),
            Region::Rom => Some(()),
            _ => None,
        }
    }

    fn inspect_read_bulk(&mut self, addr: Address, out: &mut [Byte]) -> InspectRun {
        // The smallest bank
        const BLOCK: Address = 1 << 20;
        if MAP[usize::from(self.overlay)][(addr >> 24) as usize] == Region::Ram
            && let Some(idx) = self.ram_index(addr)
        {
            let max = out.len().min((BLOCK - (addr & (BLOCK - 1))) as usize);
            out[..max].copy_from_slice(&self.ram[idx..idx + max]);
            return InspectRun::Data(max);
        }
        inspect_read_bytes(self, addr, out, BLOCK)
    }

    fn inspect_generation(&self) -> u32 {
        u32::from(self.overlay)
    }
}

impl<TRenderer> Debuggable for MduBus<TRenderer>
where
    TRenderer: Renderer,
{
    fn get_debug_properties(&self) -> crate::debuggable::DebuggableProperties {
        use crate::dbgprop_nest;
        use crate::debuggable::*;

        vec![
            dbgprop_nest!("Apple Desktop Bus", self.via1.adb),
            dbgprop_nest!("Apple Sound Chip", self.asc),
            dbgprop_nest!("RBV", self.rbv),
            dbgprop_nest!("SCSI controller (NCR 5380)", self.scsi),
            dbgprop_nest!("SWIM", self.swim),
            dbgprop_nest!("VIA 1 (SY6522)", self.via1),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::renderer::NullRenderer;

    fn bus(ram_size: usize) -> MduBus<NullRenderer> {
        MduBus::new(
            MacModel::MacIIci,
            &vec![0; 512 * 1024],
            None,
            NullRenderer::new(0, 0).unwrap(),
            MacMonitor::HiRes14,
            MouseMode::Disabled,
            Some(ram_size),
        )
    }

    fn read(bus: &mut MduBus<NullRenderer>, addr: Address) -> BusResponse {
        bus.access(BusAccess::<Bus68020>::read(addr, 1, 0)).unwrap()
    }

    fn write(bus: &mut MduBus<NullRenderer>, addr: Address, val: u32) {
        let _ = bus
            .access(BusAccess::<Bus68020>::write(addr, 1, val, 0))
            .unwrap();
    }

    fn read_byte(bus: &mut MduBus<NullRenderer>, addr: Address) -> u32 {
        match read(bus, addr) {
            BusResponse::Ack { data, .. } => data,
            r => panic!("{addr:08X}: {r:?}"),
        }
    }

    #[test]
    fn bank_split() {
        assert_eq!(bank_sizes(1 << 20), (1 << 20, 0));
        assert_eq!(bank_sizes(5 << 20), (1 << 20, 4 << 20));
        assert_eq!(bank_sizes(8 << 20), (4 << 20, 4 << 20));
        assert_eq!(bank_sizes(16 << 20), (16 << 20, 0));
        assert_eq!(bank_sizes(80 << 20), (16 << 20, 64 << 20));
        for &size in MacModel::MacIIci.ram_size_options() {
            bank_sizes(size);
        }
    }

    /// Each bank repeats in its 64 MB window, which tells the ROM its size
    #[test]
    fn banks_repeat() {
        let mut bus = bus(5 << 20);
        // Leave the overlay
        let _ = read(&mut bus, 0x4000_0000);
        write(&mut bus, 0x0000_0010, 0xA5);
        write(&mut bus, 0x0400_0010, 0x5A);
        assert_eq!(read_byte(&mut bus, 0x0010_0010), 0xA5);
        assert_eq!(read_byte(&mut bus, 0x03F0_0010), 0xA5);
        assert_eq!(read_byte(&mut bus, 0x0440_0010), 0x5A);
        assert_eq!(read_byte(&mut bus, 0x07C0_0010), 0x5A);
        assert_eq!(read(&mut bus, 0x0800_0000), BusResponse::BusError);
    }

    /// The ROM finds the MDU by VIA 1 repeating 256 KB above itself but not
    /// 128 KB, and by the RBV and the Bt478 answering
    #[test]
    fn decoder_probe() {
        let mut bus = bus(8 << 20);
        let ier = 0x50F0_1C00;
        write(&mut bus, ier, 0x7F);
        write(&mut bus, ier, 0x85);
        assert_eq!(read_byte(&mut bus, ier + 0x4_0000), 0x85);
        assert_ne!(read_byte(&mut bus, ier + 0x2_0000), 0x85);
        assert!(matches!(
            read(&mut bus, 0x50F2_6000),
            BusResponse::Ack { .. }
        ));
        assert!(matches!(
            read(&mut bus, 0x50F2_4000),
            BusResponse::Ack { .. }
        ));
    }
}
