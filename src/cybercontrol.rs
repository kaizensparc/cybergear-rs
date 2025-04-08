use crate::canusb::CANUSB;
use crate::cybergear::{CybergearMotor, MotorError};
use crate::cybergear_proto::*;
use std::collections::HashMap;

pub struct Controller {
    can: CANUSB,
    host_can_id: u8,
    motors: HashMap<u8, CybergearMotor>,
}

impl Controller {
    pub fn new(can: CANUSB, host_can_id: u8, motor_ids: Vec<u8>) -> Self {
        let mut motors = HashMap::new();
        for id in motor_ids {
            motors.insert(id, CybergearMotor::new(id));
        }

        Self {
            can,
            host_can_id,
            motors,
        }
    }

    /// Returns true if it processed something
    pub fn process_incoming(&mut self) -> bool {
        let frame: CanFrame = match self.can.get_incoming() {
            Some((ext_id, data)) => CanFrame { ext_id, data },
            None => return false,
        };

        // println!("Got frame: {:?}", frame);
        let frame: Frame = match frame.try_into() {
            Ok(x) => x,
            Err(e) => {
                eprintln!("Could not parse CAN frame: {}", e);
                return true;
            }
        };
        // println!("Got frame: {:?}", frame);

        match self.motors.get_mut(&frame.src_id) {
            Some(motor) => motor.process_incoming(&frame.message),
            None => eprintln!("Unregistered motor ID {}", frame.src_id),
        }

        return true;
    }

    pub fn get_motor_feedback(&self, motor_id: u8) -> Result<MotorFeedback, MotorError> {
        let motor = self.motors.get(&motor_id).ok_or(MotorError::Unregistered)?;
        motor.feedback.ok_or(MotorError::Timeout)
    }

    pub fn get_device_id(&mut self, motor_id: u8) -> Result<[u8; 8], MotorError> {
        // Send the request to the motor if it's not in cache
        {
            let motor = self.motors.get(&motor_id).ok_or(MotorError::Unregistered)?;

            if motor.device_id.is_none() {
                let frame: CanFrame = Frame {
                    src_id: self.host_can_id,
                    dst_id: motor_id,
                    message: Message::DeviceID([0; 8]),
                }
                .into();
                self.can.write_extended_frame(&frame.ext_id, &frame.data);
                self.process_incoming();
            }
        }

        let motor = self.motors.get(&motor_id).ok_or(MotorError::Unregistered)?;
        match motor.device_id {
            Some(did) => Ok(did.clone()),
            None => motor.device_id.ok_or(MotorError::Timeout),
        }
    }

    pub fn enable_motor(&mut self, motor_id: u8) -> Result<(), MotorError> {
        let frame: CanFrame = Frame {
            src_id: self.host_can_id,
            dst_id: motor_id,
            message: Message::EnableDevice,
        }
        .into();

        self.can.write_extended_frame(&frame.ext_id, &frame.data);
        self.process_incoming();

        Ok(())
    }

    pub fn disable_motor(&mut self, motor_id: u8) -> Result<(), MotorError> {
        let frame: CanFrame = Frame {
            src_id: self.host_can_id,
            dst_id: motor_id,
            message: Message::DisableDevice,
        }
        .into();

        self.can.write_extended_frame(&frame.ext_id, &frame.data);
        self.process_incoming();

        Ok(())
    }

    pub fn set_run_mode(&mut self, motor_id: u8, run_mode: RunMode) -> Result<(), MotorError> {
        self.write_parameter(motor_id, Parameter::RunMode(run_mode))
    }

    pub fn set_zero_position(&mut self, motor_id: u8) -> Result<(), MotorError> {
        let frame: CanFrame = Frame {
            src_id: self.host_can_id,
            dst_id: motor_id,
            message: Message::SetMechanicalZeroPosition,
        }
        .into();

        self.can.write_extended_frame(&frame.ext_id, &frame.data);
        self.process_incoming();

        Ok(())
    }

    pub fn read_parameter(&mut self, motor_id: u8, param: Parameter) -> Result<(), MotorError> {
        let frame: CanFrame = Frame {
            src_id: self.host_can_id,
            dst_id: motor_id,
            message: Message::ReadSingleParam(param),
        }
        .into();

        self.can.write_extended_frame(&frame.ext_id, &frame.data);
        self.process_incoming();

        Ok(())
    }

    pub fn write_parameter(&mut self, motor_id: u8, param: Parameter) -> Result<(), MotorError> {
        let frame: CanFrame = Frame {
            src_id: self.host_can_id,
            dst_id: motor_id,
            message: Message::WriteSingleParam(param),
        }
        .into();

        self.can.write_extended_frame(&frame.ext_id, &frame.data);
        self.process_incoming();

        Ok(())
    }

    pub fn motion_control(&mut self, motor_id: u8, mc: MotionControl) -> Result<(), MotorError> {
        let frame: CanFrame = Frame {
            src_id: self.host_can_id,
            dst_id: motor_id,
            message: Message::MotionControl(mc),
        }
        .into();

        self.can.write_extended_frame(&frame.ext_id, &frame.data);
        self.process_incoming();

        Ok(())
    }

    pub fn set_can_id(&mut self, old_can_id: u8, new_can_id: u8) -> Result<(), MotorError> {
        let frame: CanFrame = Frame {
            src_id: self.host_can_id,
            dst_id: old_can_id,
            message: Message::SetCANID(new_can_id),
        }
        .into();

        self.can.write_extended_frame(&frame.ext_id, &frame.data);
        self.process_incoming();

        let mut motor = self
            .motors
            .remove(&old_can_id)
            .ok_or(MotorError::Unregistered)?;
        motor.can_id = new_can_id;
        self.motors.insert(new_can_id, motor);

        Ok(())
    }
}
