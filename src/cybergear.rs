use crate::cybergear_proto::{Message, ModeStatus, MotorFeedback, RunMode};
use thiserror::Error;

#[derive(Error, Debug)]
pub enum MotorError {
    #[error("Timeout")]
    Timeout,
    #[error("Motor in fault: {0:?}")]
    Fault(MotorFeedback),
    #[error("Unregistered motor")]
    Unregistered,
    #[error("Unknown error")]
    Unknown,
}

pub struct Configuration {
    run_mode: RunMode,
    iq_ref: f32,
    speed_ref: f32,
    limit_torque: f32,
    cur_kp: f32,
    cur_ki: f32,
    cur_filt_gain: f32,
    loc_ref: f32,
    limit_speed: f32,
    limit_current: f32,
    mech_pos: f32,
    iq_filter: f32,
    mech_velocity: f32,
    vbus: f32,
    rotation: i16,
    loc_kp: f32,
    spd_kp: f32,
    spd_ki: f32,
}

pub struct Status {
    /// Current operational mode
    pub mode: ModeStatus,

    /// Fault indicators
    pub fault: bool,
    pub not_calibrated: bool,
    pub hall_encoding_failure: bool,
    pub magnetic_encoding_failure: bool,
    pub over_temperature: bool,
    pub over_current: bool,
    pub undervoltage_fault: bool,

    /// Sensors
    pub current_angle: f32,
    pub current_angular_velocity: f32,
    pub current_torque: f32,
    pub current_temperature: f32,
}

pub struct CybergearMotor {
    pub can_id: u8,
    pub device_id: Option<[u8; 8]>,
    pub feedback: Option<MotorFeedback>,
}

impl CybergearMotor {
    pub fn new(can_id: u8) -> Self {
        CybergearMotor {
            can_id,
            device_id: None,
            feedback: None,
        }
    }

    pub fn process_incoming(&mut self, message: &Message) {
        match message {
            Message::DeviceID(di) => {
                self.device_id = Some(di.clone());
            }
            Message::MotorFeedback(mf) => {
                self.feedback = Some(mf.clone());
            }
            Message::ReadSingleParam(p) => {
                println!("Param: {:?}", p)
            }

            Message::EnableDevice
            | Message::DisableDevice
            | Message::SetMechanicalZeroPosition
            | Message::SetCANID(_)
            | Message::WriteSingleParam(_)
            | Message::MotionControl(_)
            | Message::ErrorReport => eprintln!("Unsupported message"),
        };
    }
}
