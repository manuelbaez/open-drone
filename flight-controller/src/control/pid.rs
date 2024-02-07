pub struct PID {
    previous_error: f32,
    proportional_multiplier: f32,
    integral_multiplier: f32,
    derivative_multiplier: f32,
    previous_integral_error: f32,
}

impl PID {
    pub fn new(
        proportional_multiplier: f32,
        integral_multiplier: f32,
        derivative_multiplier: f32,
    ) -> Self {
        PID {
            previous_error: 0.0_f32,
            proportional_multiplier,
            integral_multiplier,
            derivative_multiplier,
            previous_integral_error: 0.0_f32,
        }
    }

    pub fn update(
        &mut self,
        desired_state: f32,
        measured_state: f32,
        iteration_length: f32,
    ) -> f32 {
        let error = desired_state - measured_state;

        let change_rate = (error - self.previous_error) / iteration_length;

        let proportional_output = error * self.proportional_multiplier;

        let integral_error = ((error + self.previous_integral_error) / 2.0) * iteration_length;
        let integral_output = self.integral_multiplier * integral_error;

        let derivative_output = change_rate * self.derivative_multiplier;

        self.previous_integral_error = integral_error;
        self.previous_error = error;
        proportional_output + integral_output + derivative_output
    }
}
