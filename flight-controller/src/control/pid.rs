use super::integrator::Integrator;

pub struct PID {
    previous_error: f32,
    proportional_multiplier: f32,
    integral_multiplier: f32,
    derivative_multiplier: f32,
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
        self.previous_error = error;

        let proportional_output = error * self.proportional_multiplier;
        let integral_output =
            self.integral_multiplier * ((error + self.previous_error) * iteration_length / 2.0);

        let derivative_output = change_rate * self.derivative_multiplier;

        proportional_output + integral_output + derivative_output
    }
}
