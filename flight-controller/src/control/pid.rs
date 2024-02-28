pub struct PID {
    previous_error: f32,
    proportional_multiplier: f32,
    integral_multiplier: f32,
    derivative_multiplier: f32,
    accumulated_error: f32,
    max_accumulated_error: f32,
}

impl PID {
    ///Set max_accumulated_error to zero to not limit integral
    pub fn new(
        proportional_multiplier: f32,
        integral_multiplier: f32,
        derivative_multiplier: f32,
        max_accumulated_error: f32,
    ) -> Self {
        PID {
            previous_error: 0.0_f32,
            proportional_multiplier,
            integral_multiplier,
            derivative_multiplier,
            accumulated_error: 0.0_f32,
            max_accumulated_error,
        }
    }

    pub fn get_update(
        &mut self,
        desired_state: f32,
        measured_state: f32,
        iteration_length: f32,
    ) -> f32 {
        let error = desired_state - measured_state;
        let proportional_output = error * self.proportional_multiplier;

        self.accumulated_error += ((error + self.previous_error) * iteration_length) / 2.0;

        if self.accumulated_error > 0.0 {
            if self.accumulated_error > self.max_accumulated_error {
                self.accumulated_error = self.max_accumulated_error
            } else if self.accumulated_error < -self.max_accumulated_error {
                self.accumulated_error = -self.max_accumulated_error
            }
        }

        let integral_output = self.integral_multiplier * self.accumulated_error;

        let change_rate = (error - self.previous_error) / iteration_length;
        let derivative_output = change_rate * self.derivative_multiplier;

        self.previous_error = error;

        proportional_output + integral_output + derivative_output
    }

    pub fn reset(&mut self) {
        self.accumulated_error = 0.0;
        self.previous_error = 0.0;
    }
}