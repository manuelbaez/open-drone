pub struct KalmanFilter {
    state_prediction: f64,
    prediction_uncertainty: f64,
    input_variance: f64,
    measurement_uncertainty: f64,
}

impl KalmanFilter {
    pub fn new(input_variance: f64, measurement_uncertainty: f64) -> Self {
        KalmanFilter {
            input_variance,
            measurement_uncertainty,
            state_prediction: 0.0_f64,
            prediction_uncertainty: 0.0_f64,
        }
    }
   pub fn apply_flilter_update(
        &mut self,
        current_rate: f64,
        measured_value: f64,
        t_interval_seconds: f64,
    ) {
        let mut state_prediction = self.state_prediction.clone();
        let mut prediction_uncertainty = self.prediction_uncertainty.clone();

        state_prediction = self.state_prediction + t_interval_seconds * current_rate as f64;

        prediction_uncertainty =
            prediction_uncertainty + t_interval_seconds.powf(2.0) * self.input_variance.powf(2.0);

        let kamal_gain = prediction_uncertainty
            / (prediction_uncertainty + self.measurement_uncertainty.powf(2.0));

        state_prediction =
            state_prediction + kamal_gain * (measured_value as f64 - state_prediction);

        prediction_uncertainty = (1.0_f64 - kamal_gain) * prediction_uncertainty;

        self.state_prediction = state_prediction;
        self.prediction_uncertainty = prediction_uncertainty;
    }

    pub fn get_current_state(&self) -> f64 {
        self.state_prediction
    }
    pub fn get_current_uncertainty(&self) -> f64 {
        self.prediction_uncertainty
    }
}
