//! Holds the machine to its real speed by the clock of the host.
//!
//! The emulator runs instructions in short batches. After each, the pacer
//! works out when the machine is due to have run that far, from the time it
//! has run in native ticks of its bus, and sleeps until then if the machine is
//! ahead. Native ticks count the time of the devices, so an overclocked CPU
//! runs more instructions in the same time.
//!
//! The sound card of the host plays the samples by a clock of its own, which
//! drifts from the one the pacer keeps to. The pacer follows how many blocks
//! of samples wait to be played, and runs the machine a little slower or
//! faster to hold them at half the queue. That keeps the pitch, which the
//! rate of the stream sets, unlike resampling the sound.

use std::thread;
use std::time::{Duration, Instant};

use crate::renderer::AUDIO_QUEUE_LEN;
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

/// Blocks of samples to hold in the queue to the host
const QUEUED_TARGET: f64 = (AUDIO_QUEUE_LEN / 2) as f64;

/// How much slower the machine runs per block above the target, or faster
/// below it
const TRIM_PER_BLOCK: f64 = 0.004;

/// Most the machine runs slower or faster than its real speed. The clocks of
/// a host and of its sound card are within a few hundredths of a percent of
/// each other, but the Quadra 700 makes 44169.6 samples a second into a
/// stream of 44100, 0.16% more.
const MAX_TRIM: f64 = 0.002;

/// Time over which the pacer averages the blocks queued, in seconds of the
/// machine. The queue goes up and down by a block as the machine adds one and
/// the host takes one.
const QUEUED_AVERAGE_TIME: f64 = 0.5;

#[derive(Default)]
pub struct Pacer {
    /// The native ticks at the last batch, and when the machine was due to
    /// reach them
    last: Option<(Ticks, Instant)>,
    /// Average of the blocks queued to the host, None without a stream that
    /// plays
    queued: Option<f64>,
}

impl Pacer {
    /// Forgets the time of the machine, for when it was not paced
    pub fn reset(&mut self) {
        self.last = None;
        self.queued = None;
    }

    /// Takes the time the machine has run, `ticks` of `frequency` Hz, and the
    /// blocks of samples queued to the host, and sleeps while the machine is
    /// ahead of the host's clock
    pub fn pace(&mut self, ticks: Ticks, frequency: Ticks, queued: Option<usize>) {
        let now = Instant::now();
        let due = match self.last {
            Some((last, due)) if ticks >= last => {
                let seconds = (ticks - last) as f64 / frequency as f64;
                let trim = self.trim(seconds, queued);
                due + Duration::from_secs_f64(seconds * (1.0 + trim))
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

    /// Takes the blocks queued after `seconds` more of the machine, and
    /// returns how much slower the machine is to run, or faster if negative
    fn trim(&mut self, seconds: f64, queued: Option<usize>) -> f64 {
        let Some(queued) = queued else {
            self.queued = None;
            return 0.0;
        };
        let queued = queued as f64;
        let average = match self.queued {
            Some(average) => {
                (queued - average).mul_add((seconds / QUEUED_AVERAGE_TIME).min(1.0), average)
            }
            None => queued,
        };
        self.queued = Some(average);
        ((average - QUEUED_TARGET) * TRIM_PER_BLOCK).clamp(-MAX_TRIM, MAX_TRIM)
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
            pacer.pace(ticks, FREQUENCY, None);
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
        pacer.pace(0, FREQUENCY, None);
        let start = Instant::now();
        // A second ahead, as after loading a save state
        pacer.pace(FREQUENCY, FREQUENCY, None);
        assert!(start.elapsed() < Duration::from_millis(10));
    }

    #[test]
    fn does_not_catch_up_after_a_stall() {
        let mut pacer = Pacer::default();
        pacer.pace(0, FREQUENCY, None);
        thread::sleep(Duration::from_millis(200));
        // Then the machine runs at its speed again, rather than flat out for
        // the 200 ms it lost
        let elapsed = run(&mut pacer, 0.05, 0.0001);
        assert!(elapsed >= Duration::from_millis(48), "{elapsed:?}");
    }

    #[test]
    fn trims_to_hold_the_queue() {
        let mut pacer = Pacer::default();
        assert_eq!(pacer.trim(0.001, None), 0.0);
        assert_eq!(pacer.trim(0.001, Some(AUDIO_QUEUE_LEN / 2)), 0.0);
        // A full queue slows the machine, by no more than the most
        let mut trim = 0.0;
        for _ in 0..1000 {
            trim = pacer.trim(0.001, Some(AUDIO_QUEUE_LEN));
        }
        assert_eq!(trim, MAX_TRIM);
        // An empty one speeds it up
        for _ in 0..1000 {
            trim = pacer.trim(0.001, Some(0));
        }
        assert_eq!(trim, -MAX_TRIM);
        // A quarter of a block above the target, once averaged
        for i in 0..10000 {
            let above = usize::from(i % 4 == 0);
            trim = pacer.trim(0.001, Some(AUDIO_QUEUE_LEN / 2 + above));
        }
        assert!((trim - TRIM_PER_BLOCK / 4.0).abs() < 0.0002, "{trim}");
    }
}
