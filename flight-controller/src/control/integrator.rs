pub struct Integrator {
    current_value: f64,
}

impl Integrator {
    pub fn new() -> Self {
        Integrator {
            current_value: 0.0_f64,
        }
    }

    pub fn add_new_value(&mut self, value: f64, interval_seconds: f64) {
        self.current_value += value * interval_seconds;
    }

    pub fn get_current_value(&self) -> f64 {
        self.current_value
    }
}
