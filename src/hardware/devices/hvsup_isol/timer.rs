use mono_clock::embedded_time::{clock, rate::Fraction, Clock, Instant};

pub struct Timer(u32);

impl Timer {
    pub fn new() -> Timer {
        Timer(0)
    }

    pub fn update(&mut self, elapsed: u32) {
        self.0 += elapsed;
    }
}

impl Clock for Timer {
    type T = u32;
    const SCALING_FACTOR: Fraction = Fraction::new(1, 1000);

    fn try_now(&self) -> Result<Instant<Self>, clock::Error> {
        Ok(Instant::new(self.0))
    }
}
