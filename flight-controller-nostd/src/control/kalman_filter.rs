use libm::powf;

pub struct KalmanFilter {
    state_prediction: f32,
    prediction_uncertainty: f32,
    input_variance: f32,
    measurement_uncertainty: f32,
}

impl KalmanFilter {
    pub fn new(input_variance: f32, measurement_uncertainty: f32) -> Self {
        KalmanFilter {
            input_variance,
            measurement_uncertainty,
            state_prediction: 0.0_f32,
            prediction_uncertainty: 0.0_f32,
        }
    }
    pub fn get_next_state_prediction(
        &mut self,
        current_rate: f32,
        measured_value: f32,
        t_interval_seconds: f32,
    ) -> f32 {
        let mut prediction_uncertainty = self.prediction_uncertainty.clone();

        //State prediction for change rate, this could be different for a different kind of system as this is simply
        //a model state prediction based on the inputs available(gyro, this could also be a model of the quadcopter dynamics
        //using motor thrust... but that's unnecesary and too complicated since the gyro can provide a really good measurement
        //of the system state change).
        let mut state_prediction = self.state_prediction + t_interval_seconds * current_rate as f32;

        prediction_uncertainty =
            prediction_uncertainty + powf(t_interval_seconds, 2.0) * powf(self.input_variance, 2.0);

        let kamal_gain = prediction_uncertainty
            / (prediction_uncertainty + powf(self.measurement_uncertainty,2.0));

        state_prediction =
            state_prediction + kamal_gain * (measured_value as f32 - state_prediction);

        prediction_uncertainty = (1.0_f32 - kamal_gain) * prediction_uncertainty;

        self.state_prediction = state_prediction;
        self.prediction_uncertainty = prediction_uncertainty;

        self.state_prediction
    }

    #[allow(dead_code)]
    pub fn get_current_state_prediction(&self) -> f32 {
        self.state_prediction
    }

    #[allow(dead_code)]
    pub fn get_current_uncertainty(&self) -> f32 {
        self.prediction_uncertainty
    }
}
