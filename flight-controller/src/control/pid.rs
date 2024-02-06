use super::integrator::Integrator;

pub struct PID {
    previous_value: f32,
    proportional_multiplier: f32,
    integral_multiplier: f32,
    derivative_multiplier: f32,
    error_integrator: Integrator,
}

impl PID {
    pub fn new(
        proportional_multiplier: f32,
        integral_multiplier: f32,
        derivative_multiplier: f32,
    ) -> Self {
        PID {
            previous_value: 0.0_f32,
            proportional_multiplier,
            integral_multiplier,
            derivative_multiplier,
            error_integrator: Integrator::new(),
        }
    }

    pub fn update(&mut self, desired_state: f32, measured_state: f32, iteration_length: f32) -> f32 {
        let error = desired_state - measured_state;
        let current_accumulated_error =
            self.error_integrator.add_new_value(error, iteration_length);

        let change_rate = (self.previous_value - measured_state) / iteration_length;
        self.previous_value = measured_state;

        let proportional_output = error * self.proportional_multiplier;
        let integral_output = current_accumulated_error * self.integral_multiplier;
        let derivative_output = change_rate * self.derivative_multiplier;

        proportional_output + integral_output + derivative_output
    }
}
