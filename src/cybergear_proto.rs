use std::convert::{From, TryFrom};
use std::f32::consts::PI as PIf32;
use thiserror::Error;

#[derive(Error, Debug)]
pub enum ProtoError {
    #[error("Parse error")]
    ParseError,
}

#[derive(Debug)]
pub struct CanFrame {
    pub ext_id: [u8; 4],
    pub data: [u8; 8],
}

#[derive(Debug)]
pub struct Frame {
    pub src_id: u8,
    pub dst_id: u8,
    pub message: Message,
}

impl From<Frame> for CanFrame {
    fn from(frame: Frame) -> Self {
        let mut ext_id = [
            frame.dst_id,
            frame.src_id,
            0,
            frame.message.communication_type(),
        ];
        // Some frames use extended ID to send additionnal information...
        match frame.message {
            Message::MotionControl(ref mc) => {
                let ext_id_data = mc.to_ext_id_data();
                ext_id[1] = ext_id_data[0];
                ext_id[2] = ext_id_data[1];
            }
            Message::MotorFeedback(ref mf) => {
                let ext_id_data = mf.to_ext_id_data();
                ext_id[1] = ext_id_data[0];
                ext_id[2] = ext_id_data[1];
            }
            Message::SetCANID(new_can_id) => ext_id[2] = new_can_id,
            _ => {}
        }

        CanFrame {
            ext_id,
            data: frame.message.to_can_data(),
        }
    }
}

impl TryFrom<CanFrame> for Frame {
    type Error = ProtoError;

    fn try_from(frame: CanFrame) -> Result<Self, Self::Error> {
        match frame.ext_id[3] {
            0 => Ok(Self {
                dst_id: frame.ext_id[0],
                src_id: frame.ext_id[1],
                message: Message::DeviceID(frame.data),
            }),
            1 => Ok(Self {
                dst_id: frame.ext_id[0],
                src_id: 0,
                message: Message::MotionControl(frame.into()),
            }),
            2 => Ok(Self {
                dst_id: frame.ext_id[0],
                src_id: frame.ext_id[1],
                message: Message::MotorFeedback(frame.into()),
            }),
            3 => Ok(Self {
                dst_id: frame.ext_id[0],
                src_id: frame.ext_id[1],
                message: Message::EnableDevice,
            }),
            4 => Ok(Self {
                dst_id: frame.ext_id[0],
                src_id: frame.ext_id[1],
                message: Message::DisableDevice,
            }),
            6 => Ok(Self {
                dst_id: frame.ext_id[0],
                src_id: frame.ext_id[1],
                message: Message::SetMechanicalZeroPosition,
            }),
            7 => Ok(Self {
                dst_id: frame.ext_id[0],
                src_id: frame.ext_id[1],
                message: Message::SetCANID(frame.ext_id[2]),
            }),
            17 => Ok(Self {
                dst_id: frame.ext_id[0],
                src_id: frame.ext_id[1],
                message: Message::ReadSingleParam(frame.try_into()?),
            }),
            18 => Ok(Self {
                dst_id: frame.ext_id[0],
                src_id: frame.ext_id[1],
                message: Message::WriteSingleParam(frame.try_into()?),
            }),
            21 => Ok(Self {
                // Why did they invert src and dst???
                dst_id: frame.ext_id[1],
                src_id: frame.ext_id[0],
                message: Message::ErrorReport,
            }),
            _ => Err(ProtoError::ParseError),
        }
    }
}

#[derive(Debug)]
pub enum Message {
    DeviceID([u8; 8]),
    MotionControl(MotionControl),
    MotorFeedback(MotorFeedback),
    EnableDevice,
    DisableDevice,
    SetMechanicalZeroPosition,
    SetCANID(u8),
    ReadSingleParam(Parameter),
    WriteSingleParam(Parameter),
    ErrorReport,
}

impl Message {
    pub fn communication_type(&self) -> u8 {
        match self {
            Message::DeviceID(_) => 0,
            Message::MotionControl(_) => 1,
            Message::MotorFeedback(_) => 2,
            Message::EnableDevice => 3,
            Message::DisableDevice => 4,
            Message::SetMechanicalZeroPosition => 6,
            Message::SetCANID(_) => 7,
            Message::ReadSingleParam(_) => 17,
            Message::WriteSingleParam(_) => 18,
            Message::ErrorReport => 21,
        }
    }

    pub fn to_can_data(&self) -> [u8; 8] {
        match self {
            Message::DeviceID(di) => di.clone(),
            Message::MotionControl(mc) => mc.to_can_data(),
            Message::MotorFeedback(mf) => mf.to_can_data(),
            Message::EnableDevice => [0; 8],
            Message::DisableDevice => [0; 8],
            Message::SetMechanicalZeroPosition => [1, 0, 0, 0, 0, 0, 0, 0],
            Message::SetCANID(_) => [1, 0, 0, 0, 0, 0, 0, 0],
            Message::ReadSingleParam(rsp) => rsp.to_can_data(),
            Message::WriteSingleParam(wsp) => wsp.to_can_data(),
            Message::ErrorReport => [0; 8],
        }
    }
}

#[repr(u8)]
#[derive(Debug, Clone, Copy)]
pub enum RunMode {
    OperationControl = 0,
    Position = 1,
    Speed = 2,
    Current = 3,
}

impl TryFrom<u8> for RunMode {
    type Error = ProtoError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::OperationControl),
            1 => Ok(Self::Position),
            2 => Ok(Self::Speed),
            3 => Ok(Self::Current),
            _ => Err(ProtoError::ParseError),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum Parameter {
    RunMode(RunMode),
    IqRef(f32),
    SpdRef(f32),
    LimitTorque(f32),
    CurKp(f32),
    CurKi(f32),
    CurFiltGain(f32),
    LocRef(f32),
    LimitSpd(f32),
    LimitCur(f32),
    MechPos(f32),
    IqFilter(f32),
    MechVeloc(f32),
    Vbus(f32),
    Rotation(i16),
    LocKp(f32),
    SpdKp(f32),
    SpdKi(f32),
}

fn make_payload_i16(param_code: u16, param_value: &i16) -> [u8; 8] {
    let code = param_code.to_le_bytes();
    let value = param_value.to_le_bytes();
    [code[0], code[1], 0x00, 0x00, 0x00, 0x00, value[0], value[1]]
}

fn make_payload_f32(param_code: u16, param_value: &f32) -> [u8; 8] {
    let code = param_code.to_le_bytes();
    let value = param_value.to_le_bytes();
    [
        code[0], code[1], 0x00, 0x00, value[0], value[1], value[2], value[3],
    ]
}

impl TryFrom<CanFrame> for Parameter {
    type Error = ProtoError;

    fn try_from(frame: CanFrame) -> Result<Self, Self::Error> {
        let index = u16::from_le_bytes([frame.data[0], frame.data[1]]);
        let float_param =
            f32::from_le_bytes([frame.data[4], frame.data[5], frame.data[6], frame.data[7]]);
        let i16_param = i16::from_be_bytes([frame.data[4], frame.data[5]]);

        match index {
            0x7005 => Ok(Self::RunMode(frame.data[4].try_into()?)),
            0x7006 => Ok(Self::IqRef(float_param)),
            0x700A => Ok(Self::SpdRef(float_param)),
            0x700B => Ok(Self::LimitTorque(float_param)),
            0x7010 => Ok(Self::CurKp(float_param)),
            0x7011 => Ok(Self::CurKi(float_param)),
            0x7014 => Ok(Self::CurFiltGain(float_param)),
            0x7016 => Ok(Self::LocRef(float_param)),
            0x7017 => Ok(Self::LimitSpd(float_param)),
            0x7018 => Ok(Self::LimitCur(float_param)),
            0x7019 => Ok(Self::MechPos(float_param)),
            0x701A => Ok(Self::IqFilter(float_param)),
            0x701B => Ok(Self::MechVeloc(float_param)),
            0x701C => Ok(Self::Vbus(float_param)),
            0x701D => Ok(Self::Rotation(i16_param)),
            0x701E => Ok(Self::LocKp(float_param)),
            0x701F => Ok(Self::SpdKp(float_param)),
            0x7020 => Ok(Self::SpdKi(float_param)),
            _ => Err(ProtoError::ParseError),
        }
    }
}

impl Parameter {
    pub fn to_can_data(&self) -> [u8; 8] {
        match self {
            Self::RunMode(runmode) => [0x05, 0x70, 0x00, 0x00, (*runmode) as u8, 0x00, 0x00, 0x00],
            Self::IqRef(f) => make_payload_f32(0x7006, f),
            Self::SpdRef(f) => make_payload_f32(0x700A, f),
            Self::LimitTorque(f) => make_payload_f32(0x700B, f),
            Self::CurKp(f) => make_payload_f32(0x7010, f),
            Self::CurKi(f) => make_payload_f32(0x7011, f),
            Self::CurFiltGain(f) => make_payload_f32(0x7014, f),
            Self::LocRef(f) => make_payload_f32(0x7016, f),
            Self::LimitSpd(f) => make_payload_f32(0x7017, f),
            Self::LimitCur(f) => make_payload_f32(0x7018, f),
            Self::MechPos(f) => make_payload_f32(0x7019, f),
            Self::IqFilter(f) => make_payload_f32(0x701A, f),
            Self::MechVeloc(f) => make_payload_f32(0x701B, f),
            Self::Vbus(f) => make_payload_f32(0x701C, f),
            Self::Rotation(i) => make_payload_i16(0x701D, i),
            Self::LocKp(f) => make_payload_f32(0x701E, f),
            Self::SpdKp(f) => make_payload_f32(0x701F, f),
            Self::SpdKi(f) => make_payload_f32(0x7020, f),
        }
    }
}

#[derive(Debug, Clone)]
pub struct MotionControl {
    /// Torque control, within -12N.m and +12N.m, clamped
    pub torque: f32,
    /// Target angle in rad, within -4pi/+4pi, clamped
    pub angle: f32,
    /// Target velocity in rad/s, within -30/+30 rad/s, clamped
    pub velocity: f32,
    /// P factor, within 0.0/500.0, clamped
    pub kp: f32,
    /// D factor, within 0.0/5.0, clamped
    pub kd: f32,
}

fn u16_to_float(value: u16, min: f32, max: f32) -> f32 {
    (value as f32) * ((max - min) / u16::MAX as f32) + min
}

fn float_to_u16(value: f32, min: f32, max: f32) -> u16 {
    ((value.clamp(min, max) - min) * (u16::MAX as f32 / (max - min))) as u16
}

#[test]
fn test_float_u16() {
    assert!(float_to_u16(-10f32, -10f32, 10f32) == 0);
    assert!(float_to_u16(10f32, -10f32, 10f32) == u16::MAX);
    assert!(float_to_u16(0f32, -10f32, 10f32) == u16::MAX / 2);

    assert!(u16_to_float(0, 0f32, 10f32) == 0f32);
    assert!(u16_to_float(u16::MAX, 0f32, 10f32) == 10f32);

    assert!(
        (u16_to_float(float_to_u16(-10f32, -10f32, 10f32), -10f32, 10f32) + 10f32).abs() < 0.5f32
    );
}

impl From<CanFrame> for MotionControl {
    fn from(frame: CanFrame) -> Self {
        let torque = u16::from_be_bytes([frame.ext_id[1], frame.ext_id[2]]);
        let angle = u16::from_be_bytes([frame.data[0], frame.data[1]]);
        let velocity = u16::from_be_bytes([frame.data[2], frame.data[3]]);
        let kp = u16::from_be_bytes([frame.data[4], frame.data[5]]);
        let kd = u16::from_be_bytes([frame.data[6], frame.data[7]]);

        let torque = u16_to_float(torque, -12f32, 12f32);
        let angle = u16_to_float(angle, -4f32 * PIf32, 4f32 * PIf32);
        let velocity = u16_to_float(velocity, -30f32, 30f32);
        let kp = u16_to_float(kp, 0f32, 500f32);
        let kd = u16_to_float(kd, 0f32, 5f32);

        Self {
            torque,
            angle,
            velocity,
            kp,
            kd,
        }
    }
}

impl MotionControl {
    pub fn to_ext_id_data(&self) -> [u8; 2] {
        float_to_u16(self.torque, -12f32, 12f32).to_be_bytes()
    }

    pub fn to_can_data(&self) -> [u8; 8] {
        let angle = float_to_u16(self.angle, -4f32 * PIf32, 4f32 * PIf32).to_be_bytes();
        let velocity = float_to_u16(self.velocity, -30f32, 30f32).to_be_bytes();
        let kp = float_to_u16(self.kp, 0f32, 500f32).to_be_bytes();
        let kd = float_to_u16(self.kd, 0f32, 5f32).to_be_bytes();

        [
            angle[0],
            angle[1],
            velocity[0],
            velocity[1],
            kp[0],
            kp[1],
            kd[0],
            kd[1],
        ]
    }
}

#[derive(Debug, Copy, Clone)]
#[repr(u8)]
pub enum ModeStatus {
    Reset = 0,
    Calibration = 1,
    Run = 2,
}

#[derive(Debug, Copy, Clone)]
pub struct MotorFeedback {
    pub fault: bool,
    pub not_calibrated: bool,
    pub hall_encoding_failure: bool,
    pub magnetic_encoding_failure: bool,
    pub over_temperature: bool,
    pub over_current: bool,
    pub undervoltage_fault: bool,
    pub mode: ModeStatus,
    pub current_angle: f32,
    pub current_angular_velocity: f32,
    pub current_torque: f32,
    pub current_temperature: f32,
}

impl std::convert::From<CanFrame> for MotorFeedback {
    fn from(frame: CanFrame) -> Self {
        let flt_byte = frame.ext_id[2];
        let angle = u16_to_float(
            u16::from_be_bytes([frame.data[0], frame.data[1]]),
            -4f32 * PIf32,
            4f32 * PIf32,
        );
        let velocity = u16_to_float(
            u16::from_be_bytes([frame.data[2], frame.data[3]]),
            -30f32,
            30f32,
        );
        let torque = u16_to_float(
            u16::from_be_bytes([frame.data[4], frame.data[5]]),
            -12f32,
            12f32,
        );
        Self {
            fault: flt_byte & 0x3F != 0,
            not_calibrated: (flt_byte & (1 << 5)) == 1,
            hall_encoding_failure: (flt_byte & (1 << 4)) == 1,
            magnetic_encoding_failure: (flt_byte & (1 << 3)) == 1,
            over_temperature: (flt_byte & (1 << 2)) == 1,
            over_current: (flt_byte & (1 << 1)) == 1,
            undervoltage_fault: (flt_byte & (1 << 0)) == 1,
            mode: match (flt_byte & (0xC0)) >> 6 {
                2 => ModeStatus::Run,
                1 => ModeStatus::Calibration,
                0 | _ => ModeStatus::Reset,
            },
            current_angle: angle,
            current_angular_velocity: velocity,
            current_torque: torque,
            current_temperature: (u16::from_be_bytes([frame.data[6], frame.data[7]]) as f32) / 10.0,
        }
    }
}

impl MotorFeedback {
    pub fn to_ext_id_data(&self) -> [u8; 2] {
        let flt_byte = 0
            | if self.undervoltage_fault { 1 } else { 0 }
            | if self.over_current { 1 << 1 } else { 0 }
            | if self.over_temperature { 1 << 2 } else { 0 }
            | if self.magnetic_encoding_failure {
                1 << 3
            } else {
                0
            }
            | if self.hall_encoding_failure {
                1 << 4
            } else {
                0
            }
            | if self.not_calibrated { 1 << 5 } else { 0 }
            | if self.fault { 1 << 6 } else { 0 };

        [flt_byte, 0]
    }

    pub fn to_can_data(&self) -> [u8; 8] {
        let angle = float_to_u16(self.current_angle, -4f32 * PIf32, 4f32 * PIf32).to_be_bytes();
        let velocity = float_to_u16(self.current_angular_velocity, -30f32, 30f32).to_be_bytes();
        let torque = float_to_u16(self.current_torque, -12f32, 12f32).to_be_bytes();
        let temperature = ((self.current_temperature / 10f32) as u16).to_be_bytes();

        [
            angle[0],
            angle[1],
            velocity[0],
            velocity[1],
            torque[0],
            torque[1],
            temperature[0],
            temperature[1],
        ]
    }
}
