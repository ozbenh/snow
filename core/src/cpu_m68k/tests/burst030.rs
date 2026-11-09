//! 68030 burst fills: the CPU goes on with the long word it asked for while
//! the rest of the line comes in (MC68030UM 6.1.3.2)

use crate::bus::Address;
use crate::bus::access::Bus68020;
use crate::bus::testbus::Testbus;
use crate::cpu_m68k::regs::RegisterCACR;
use crate::cpu_m68k::{CpuM68030, FPU_M68882, M68030_ADDRESS_MASK};
use crate::tickable::Ticks;
use crate::types::Word;

type TestCpu = CpuM68030<Testbus<Address, u8, Bus68020>>;

const CODE: Address = 0x1000;
const DATA: Address = 0x4000;

/// Wait states of the burst cycles after the first: two clocks each
const BURST_WAITS: Ticks = 1;

/// Clocks of the three burst cycles after the first
const BURST_TAIL: Ticks = 3 * (1 + BURST_WAITS);

/// MOVE.L (A0),D1
const MOVE_L_A0_D1: Word = 0x2210;
/// MOVE.L d16(A0),D3
const MOVE_L_D16_A0_D3: Word = 0x2628;
/// MOVEQ #0,D2
const MOVEQ_0_D2: Word = 0x7400;
/// LEA 16(A0),A0
const LEA_16_A0: [Word; 2] = [0x41E8, 0x0010];
/// DBF D7,<disp>
const DBF_D7: Word = 0x51CF;

/// A loop that reads a new line of the data cache each pass, with the
/// instructions `body` after the read
fn loop_code(body: &[Word]) -> Vec<Word> {
    let mut code = vec![MOVE_L_A0_D1];
    code.extend_from_slice(body);
    code.extend_from_slice(&LEA_16_A0);
    code.push(DBF_D7);
    // From the extension word back to the start
    let disp = -2 * code.len() as i32;
    code.push(disp as Word);
    code
}

fn testcpu(code: &[Word], burst: bool) -> TestCpu {
    let mut bus = Testbus::new(M68030_ADDRESS_MASK);
    if burst {
        bus.burst_waits = Some(BURST_WAITS);
    }
    let mut cpu = TestCpu::new(bus, FPU_M68882);
    for (i, &word) in code.iter().enumerate() {
        let addr = CODE + 2 * i as Address;
        cpu.bus.mem.insert(addr, (word >> 8) as u8);
        cpu.bus.mem.insert(addr + 1, word as u8);
    }
    // Both caches on, burst fills of the data cache only
    cpu.regs.cacr = RegisterCACR(0x1101);
    cpu.regs.write_a::<Address>(0, DATA);
    cpu.regs.d[7] = 100;
    cpu.set_pc(CODE).unwrap();
    cpu.prefetch_refill().unwrap();
    cpu
}

/// Runs one pass of the loop, returns its clocks
fn run_pass(cpu: &mut TestCpu) -> Ticks {
    let start = cpu.cycles;
    loop {
        cpu.step().unwrap();
        if cpu.regs.pc == CODE {
            return cpu.cycles - start;
        }
    }
}

/// Runs two passes, after which the loop is in the instruction cache
fn warm_up(cpu: &mut TestCpu) {
    run_pass(cpu);
    run_pass(cpu);
}

#[test]
fn burst_overlaps_what_follows() {
    let code = loop_code(&[MOVEQ_0_D2; 8]);
    let mut single = testcpu(&code, false);
    let mut burst = testcpu(&code, true);
    warm_up(&mut single);
    warm_up(&mut burst);

    // The miss reads one long word, or the whole line with a burst whose
    // last three cycles run during the MOVEQs
    assert_eq!(run_pass(&mut burst), run_pass(&mut single));
}

#[test]
fn no_credit_without_a_burst() {
    let mut cpu = testcpu(&loop_code(&[MOVEQ_0_D2]), false);
    warm_up(&mut cpu);
    cpu.step().unwrap();
    assert_eq!(cpu.bus_credit, 0);
    assert_eq!(cpu.burst_line, None);
}

#[test]
fn hit_on_the_line_being_filled_waits() {
    let mut cpu = testcpu(&loop_code(&[MOVE_L_D16_A0_D3, 4]), true);
    warm_up(&mut cpu);
    let line = cpu.regs.read_a::<Address>(0);

    // MOVE.L (A0),D1: a miss, which fills the line with a burst
    cpu.step().unwrap();
    assert_eq!(cpu.burst_line, Some((false, line)));
    assert!(cpu.bus_credit > 0 && cpu.bus_credit <= BURST_TAIL);

    // MOVE.L 4(A0),D3: a hit on that line, which waits for the burst
    cpu.step().unwrap();
    assert_eq!(cpu.bus_credit, 0);
    assert_eq!(cpu.burst_line, None);
}

#[test]
fn hit_on_another_line_goes_on() {
    let mut cpu = testcpu(&loop_code(&[MOVE_L_D16_A0_D3, -16i16 as Word]), true);
    warm_up(&mut cpu);
    let line = cpu.regs.read_a::<Address>(0);

    cpu.step().unwrap();
    let credit = cpu.bus_credit;
    assert!(credit > 0);

    // MOVE.L -16(A0),D3: a hit on the line of the previous pass, which
    // overlaps the burst
    cpu.step().unwrap();
    assert!(cpu.bus_credit < credit);
    assert_eq!(cpu.burst_line, Some((false, line)));
}
