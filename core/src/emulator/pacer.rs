//! Holds the machine to its real speed by the clock of the host.
//!
//! The emulator runs instructions in short batches. After each, the pacer
//! works out when the machine is due to have run that far, from the time it
//! has run in native ticks of its bus, and sleeps until then if the machine is
//! ahead. Native ticks count the time of the devices, so an overclocked CPU
//! runs more instructions in the same time.

use std::thread;
use std::time::{Duration, Instant};

use crate::tickable::Ticks;

/// The pacer sleeps only once the machine is this far ahead, so it sleeps
/// about a thousand times a second at most, whatever the machine
const MIN_SLEEP: Duration = Duration::from_millis(1);

/// How far the machine can fall behind before the pacer gives up catching
/// up, as after the host was busy elsewhere
const MAX_BEHIND: Duration = Duration::from_millis(50);

/// How far ahead the machine can be: further, its time jumped, as when a save
/// state is loaded
const MAX_AHEAD: Duration = Duration::from_millis(100);

#[derive(Default)]
pub struct Pacer {
    /// The native ticks at the last batch, and when the machine was due to
    /// reach them
    last: Option<(Ticks, Instant)>,
}

impl Pacer {
    /// Forgets the time of the machine, for when it was not paced
    pub fn reset(&mut self) {
        self.last = None;
    }

    /// Takes the time the machine has run, `ticks` of `frequency` Hz, and
    /// sleeps while it is ahead of the host's clock
    pub fn pace(&mut self, ticks: Ticks, frequency: Ticks) {
        let now = Instant::now();
        let due = match self.last {
            Some((last, due)) if ticks >= last => {
                due + Duration::from_secs_f64((ticks - last) as f64 / frequency as f64)
            }
            _ => now,
        };
        // Start again from now rather than run flat out to catch up, or wait
        // for a time that jumped
        let due = if now > due + MAX_BEHIND || due > now + MAX_AHEAD {
            now
        } else {
            due
        };
        self.last = Some((ticks, due));

        let ahead = due.saturating_duration_since(now);
        if ahead >= MIN_SLEEP {
            thread::sleep(ahead);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FREQUENCY: Ticks = 7_833_600;

    /// Runs `seconds` of a machine in batches of `batch` seconds that take no
    /// time on the host. Returns the time it took.
    fn run(pacer: &mut Pacer, seconds: f64, batch: f64) -> Duration {
        let start = Instant::now();
        let batch = (FREQUENCY as f64 * batch) as Ticks;
        let mut ticks = 0;
        while ticks < (FREQUENCY as f64 * seconds) as Ticks {
            ticks += batch;
            pacer.pace(ticks, FREQUENCY);
        }
        start.elapsed()
    }

    #[test]
    fn keeps_to_the_clock() {
        let elapsed = run(&mut Pacer::default(), 0.1, 0.0001);
        assert!(elapsed >= Duration::from_millis(98), "{elapsed:?}");
        assert!(elapsed < Duration::from_millis(130), "{elapsed:?}");
    }

    #[test]
    fn does_not_wait_for_a_jump() {
        let mut pacer = Pacer::default();
        pacer.pace(0, FREQUENCY);
        let start = Instant::now();
        // A second ahead, as after loading a save state
        pacer.pace(FREQUENCY, FREQUENCY);
        assert!(start.elapsed() < Duration::from_millis(10));
    }

    #[test]
    fn does_not_catch_up_after_a_stall() {
        let mut pacer = Pacer::default();
        pacer.pace(0, FREQUENCY);
        thread::sleep(Duration::from_millis(200));
        // Then the machine runs at its speed again, rather than flat out for
        // the 200 ms it lost
        let elapsed = run(&mut pacer, 0.05, 0.0001);
        assert!(elapsed >= Duration::from_millis(48), "{elapsed:?}");
    }
}
