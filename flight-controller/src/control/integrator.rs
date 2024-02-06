pub struct Integrator {
    current_value: f32,
}

impl Integrator {
    pub fn new() -> Self {
        Integrator {
            current_value: 0.0_f32,
        }
    }

    pub fn add_new_value(&mut self, value: f32, interval_seconds: f32) ->f32 {
        self.current_value += value * interval_seconds;
        self.current_value
    }

    pub fn get_current_value(&self) -> f32 {
        self.current_value
    }
}
