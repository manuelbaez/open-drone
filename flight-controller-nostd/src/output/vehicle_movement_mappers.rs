use core::ops::Add;

use crate::util::math::vectors::RotationVector3D;

pub enum VehicleTypesMapper {
    Quadcopter,
}

pub trait FlyingVehicleMovementMapper<TActuator> {
    fn map_controller_output_to_actuators_input(
        &self,
        throttle: f32,
        rotation_input: RotationVector3D,
    ) -> TActuator;
}

pub struct QuadcoperActuatorsValues {
    pub motor_front_left: f32,  //CW
    pub motor_front_right: f32, //CCW
    pub motor_rear_left: f32,   //CCW
    pub motor_rear_right: f32,  //CW
}

impl From<f32> for QuadcoperActuatorsValues {
    fn from(value: f32) -> Self {
        Self {
            motor_front_left: value,
            motor_rear_left: value,
            motor_front_right: value,
            motor_rear_right: value,
        }
    }
}

impl Add for QuadcoperActuatorsValues {
    type Output = QuadcoperActuatorsValues;

    fn add(self, other: Self) -> Self::Output {
        Self {
            motor_front_left: self.motor_front_left + other.motor_front_left,
            motor_rear_left: self.motor_rear_left + other.motor_rear_left,
            motor_front_right: self.motor_front_right + other.motor_front_right,
            motor_rear_right: self.motor_rear_right + other.motor_rear_right,
        }
    }
}

impl QuadcoperActuatorsValues {
    fn constraint_value(value: f32, min: f32, max: f32) -> f32 {
        if value > max {
            return max;
        }
        if value < min {
            return min;
        }
        value
    }

    fn get_constrained_to_range(&self, min: f32, max: f32) -> Self {
        Self {
            motor_front_left: Self::constraint_value(self.motor_front_left, min, max),
            motor_front_right: Self::constraint_value(self.motor_front_right, min, max),
            motor_rear_left: Self::constraint_value(self.motor_rear_left, min, max),
            motor_rear_right: Self::constraint_value(self.motor_rear_right, min, max),
        }
    }
}

pub struct Quadcopter {
    motor_min_power: f32,
    motor_max_power: f32,
}

impl Quadcopter {
    pub fn new(motor_min_power: f32, motor_max_power: f32) -> Self {
        Self {
            motor_min_power,
            motor_max_power,
        }
    }

    fn map_roll_to_motor_input(&self, roll_value: f32) -> QuadcoperActuatorsValues {
        QuadcoperActuatorsValues {
            motor_front_left: roll_value,
            motor_rear_left: roll_value,
            motor_front_right: -roll_value,
            motor_rear_right: -roll_value,
        }
    }

    fn map_pitch_to_motor_input(&self, pitch_value: f32) -> QuadcoperActuatorsValues {
        QuadcoperActuatorsValues {
            motor_front_left: pitch_value,
            motor_rear_left: -pitch_value,
            motor_front_right: pitch_value,
            motor_rear_right: -pitch_value,
        }
    }

    fn map_yaw_to_motor_input(&self, yaw_value: f32) -> QuadcoperActuatorsValues {
        QuadcoperActuatorsValues {
            motor_front_left: yaw_value,
            motor_rear_left: -yaw_value,
            motor_front_right: -yaw_value,
            motor_rear_right: yaw_value,
        }
    }
}

impl FlyingVehicleMovementMapper<QuadcoperActuatorsValues> for Quadcopter {
    fn map_controller_output_to_actuators_input(
        &self,
        throttle: f32,
        rotation_input: RotationVector3D,
    ) -> QuadcoperActuatorsValues {
        let motors_throttle = QuadcoperActuatorsValues::from(throttle);
        let pitch_input = self.map_pitch_to_motor_input(rotation_input.pitch);
        let roll_input = self.map_roll_to_motor_input(rotation_input.roll);
        let yaw_input = self.map_yaw_to_motor_input(rotation_input.yaw);
        let combined_output = motors_throttle + pitch_input + roll_input + yaw_input;
        combined_output.get_constrained_to_range(
            self.motor_min_power.clone(),
            self.motor_max_power.clone(),
        )
    }
}
