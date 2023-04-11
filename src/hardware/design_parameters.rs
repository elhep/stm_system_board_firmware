use stm32h7xx_hal::time::MegaHertz;

/// The system clock, used in various timer calculations
pub const SYSCLK: MegaHertz = MegaHertz(400);

/// The optimal counting frequency of the hardware timers used for timestamping and sampling.
pub const TIMER_FREQUENCY: MegaHertz = MegaHertz(100);
//pub const TIMER_PERIOD: f32 = 1. / ((TIMER_FREQUENCY.0 * 1_000_000) as f32);

