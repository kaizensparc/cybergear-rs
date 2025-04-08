use anyhow::Result;
use std::time::Duration;

mod canusb;
mod cybercontrol;
mod cybergear;
mod cybergear_proto;

use cybercontrol::Controller;

const MOT_FRNT: u8 = 127;
const MOT_BACK: u8 = 64;

fn main() -> Result<()> {
    println!("Serial port open");

    let can = canusb::CANUSB::new("/dev/ttyUSB0");

    let mut motors = cybercontrol::Controller::new(can, 1, vec![MOT_FRNT, MOT_BACK]);

    println!("Get device IDs");
    let device_id_front = motors.get_device_id(MOT_FRNT)?;
    let device_id_back = motors.get_device_id(MOT_BACK)?;
    println!(
        "Motor device ID: {:X?}, {:X?}",
        device_id_back, device_id_front
    );

    println!("Disable motors");
    motors.disable_motor(MOT_BACK)?;
    println!("Disable motors");
    motors.disable_motor(MOT_FRNT)?;

    println!("Calibrate zero");
    std::thread::sleep(Duration::from_secs(1));
    motors.set_zero_position(MOT_BACK)?;
    motors.set_zero_position(MOT_FRNT)?;
    std::thread::sleep(Duration::from_secs(1));

    println!("Enable motors");
    motors.enable_motor(MOT_BACK)?;
    motors.enable_motor(MOT_FRNT)?;

    println!("Go to origin");
    motors.set_run_mode(MOT_BACK, cybergear_proto::RunMode::Position)?;
    motors.set_run_mode(MOT_FRNT, cybergear_proto::RunMode::Position)?;
    motors.write_parameter(MOT_BACK, cybergear_proto::Parameter::LocRef(0f32))?;
    motors.write_parameter(MOT_FRNT, cybergear_proto::Parameter::LocRef(0f32))?;
    std::thread::sleep(Duration::from_secs(1));

    leader_follower_position(&mut motors)?;

    println!("Disable motors");
    motors.disable_motor(MOT_BACK)?;
    motors.disable_motor(MOT_FRNT)?;

    Ok(())
}

fn position(motors: &mut Controller) -> Result<()> {
    motors.write_parameter(MOT_BACK, cybergear_proto::Parameter::LimitSpd(30f32))?;
    motors.write_parameter(MOT_FRNT, cybergear_proto::Parameter::LimitSpd(30f32))?;
    loop {
        motors.write_parameter(MOT_BACK, cybergear_proto::Parameter::LocRef(0f32))?;
        motors.write_parameter(MOT_FRNT, cybergear_proto::Parameter::LocRef(0f32))?;
        std::thread::sleep(Duration::from_secs(1));
        motors.write_parameter(
            MOT_BACK,
            cybergear_proto::Parameter::LocRef(720f32.to_radians()),
        )?;
        motors.write_parameter(
            MOT_FRNT,
            cybergear_proto::Parameter::LocRef(720f32.to_radians()),
        )?;
        std::thread::sleep(Duration::from_secs(1));
        motors.write_parameter(
            MOT_BACK,
            cybergear_proto::Parameter::LocRef(-720f32.to_radians()),
        )?;
        motors.write_parameter(
            MOT_FRNT,
            cybergear_proto::Parameter::LocRef(-720f32.to_radians()),
        )?;
        std::thread::sleep(Duration::from_secs(1));
    }
}

fn speed(motors: &mut Controller) -> Result<()> {
    println!("Set speed mode");
    motors.write_parameter(MOT_BACK, cybergear_proto::Parameter::SpdRef(0f32))?;
    motors.write_parameter(MOT_FRNT, cybergear_proto::Parameter::SpdRef(0f32))?;
    motors.set_run_mode(MOT_BACK, cybergear_proto::RunMode::Speed)?;
    motors.set_run_mode(MOT_FRNT, cybergear_proto::RunMode::Speed)?;
    std::thread::sleep(Duration::from_secs(1));

    loop {
        motors.write_parameter(MOT_BACK, cybergear_proto::Parameter::SpdRef(30f32))?;
        motors.write_parameter(MOT_FRNT, cybergear_proto::Parameter::SpdRef(30f32))?;
        std::thread::sleep(Duration::from_secs(5));
        motors.write_parameter(MOT_BACK, cybergear_proto::Parameter::SpdRef(-30f32))?;
        motors.write_parameter(MOT_FRNT, cybergear_proto::Parameter::SpdRef(-30f32))?;
        std::thread::sleep(Duration::from_secs(5));
        motors.write_parameter(MOT_BACK, cybergear_proto::Parameter::SpdRef(0f32))?;
        motors.write_parameter(MOT_FRNT, cybergear_proto::Parameter::SpdRef(0f32))?;
        std::thread::sleep(Duration::from_secs(5));
    }
}

fn rotation(motors: &mut Controller) -> Result<()> {
    for i in 1..360 {
        motors.write_parameter(
            MOT_FRNT,
            cybergear_proto::Parameter::LocRef((i as f32).to_radians()),
        )?;
    }
    Ok(())
}

fn leader_follower_position(motors: &mut Controller) -> Result<()> {
    println!("Set current mode");
    motors.set_run_mode(MOT_BACK, cybergear_proto::RunMode::Current)?;
    motors.set_run_mode(MOT_FRNT, cybergear_proto::RunMode::Position)?;
    motors.write_parameter(MOT_FRNT, cybergear_proto::Parameter::LimitSpd(30f32))?;
    motors.write_parameter(MOT_FRNT, cybergear_proto::Parameter::LimitCur(3f32))?;
    std::thread::sleep(Duration::from_secs(1));

    println!("Operationnal");
    loop {
        motors.write_parameter(MOT_BACK, cybergear_proto::Parameter::LocRef(0f32))?;
        let mot_fb_back = motors.get_motor_feedback(MOT_BACK)?;

        motors.write_parameter(
            MOT_FRNT,
            cybergear_proto::Parameter::LocRef(mot_fb_back.current_angle),
        )?;
    }
}

fn leader_follower_current(motors: &mut Controller) -> Result<()> {
    println!("Set current mode");
    motors.set_run_mode(MOT_BACK, cybergear_proto::RunMode::Current)?;
    motors.set_run_mode(MOT_FRNT, cybergear_proto::RunMode::Current)?;
    std::thread::sleep(Duration::from_secs(1));

    println!("Operationnal");
    let mut currents = [0f32, 0f32];
    loop {
        motors.write_parameter(MOT_BACK, cybergear_proto::Parameter::IqRef(0f32))?;
        motors.write_parameter(MOT_FRNT, cybergear_proto::Parameter::IqRef(currents[1]))?;

        let mot_fb_front = motors.get_motor_feedback(MOT_FRNT)?;
        let mot_fb_back = motors.get_motor_feedback(MOT_BACK)?;
        currents[0] = (mot_fb_front.current_angle - mot_fb_back.current_angle) * 1f32;
        currents[1] = (mot_fb_back.current_angle - mot_fb_front.current_angle) * 1f32;

        currents[0] = currents[0].clamp(-3f32, 3f32);
        currents[1] = currents[1].clamp(-3f32, 3f32);
        //println!("{:?}", currents[0]);
        //motors.read_parameter(MOT_FRNT, cybergear_proto::Parameter::Rotation(0))?;
    }
}
