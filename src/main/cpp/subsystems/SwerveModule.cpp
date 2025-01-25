// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModule.h"

#include <units/length.h>
#include <units/velocity.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>

#include <frc/geometry/Rotation2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace SC;
using namespace SwerveConstants::DrivetrainConstants;

using namespace units;

using namespace ctre::phoenix6;

using namespace frc;


SwerveModule::SwerveModule(SC_SwerveConfigs corner, SC_SwervePID pid_struct) 
        : _drive_motor(corner.CAN_ID),
        _steer_motor(corner.SteerMotorPort),
        _steer_encoder(corner.EncoderPort),
        _drive_feed_forward{pid_struct.S, pid_struct.V, pid_struct.A}
{
    // Load motor configs
    SC::SC_SwerveCurrents _swerve_current_constants;

    // Create and apply drive configs
    configs::CurrentLimitsConfigs drive_current_limit{};
    drive_current_limit
        .WithSupplyCurrentLimitEnable(_swerve_current_constants.Current_Limit_Enable)
        .WithSupplyCurrentLimit(_swerve_current_constants.Current_Limit_Drive)
        .WithSupplyCurrentLowerLimit(_swerve_current_constants.Drive_Current_Threshold)
        .WithSupplyCurrentLowerTime(_swerve_current_constants.Drive_Current_Time);

    _drive_motor_config.CurrentLimits = drive_current_limit;
    _drive_motor_config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25_s;
    _drive_motor.GetConfigurator().Apply(_drive_motor_config);
    SetBrakeMode();
    ResetEncoder();

    // Create and apply steer configs
    configs::CurrentLimitsConfigs steer_current_limit{};
    steer_current_limit
        .WithSupplyCurrentLimitEnable(_swerve_current_constants.Current_Limit_Enable)
        .WithSupplyCurrentLimit(_swerve_current_constants.Current_Limit_Steer)
        .WithSupplyCurrentLowerLimit(_swerve_current_constants.Steer_Current_Threshold)
        .WithSupplyCurrentLowerTime(_swerve_current_constants.Steer_Current_Time);


    _steer_motor_config.CurrentLimits = steer_current_limit;
    _steer_motor_config.MotorOutput.Inverted = _swerve_current_constants.Steer_Motor_Reversed;
    _steer_motor_config.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;

    _steer_motor.GetConfigurator().Apply(_steer_motor_config);

    //Create and apply encoder configs
    configs::MagnetSensorConfigs encoder_magnet_config{};
    encoder_magnet_config
        .WithMagnetOffset(corner.EncoderOffset)
        .WithSensorDirection(_swerve_current_constants.Encoder_Reversed)
        .WithAbsoluteSensorDiscontinuityPoint(180_deg);
    
    _encoder_config.MagnetSensor = encoder_magnet_config;
    _steer_encoder.GetConfigurator().Apply(_encoder_config);

    _steer_pid_controller.EnableContinuousInput(-180_deg, 180_deg);

    _drive_pid_controller.SetP(pid_struct.Kp);
    _drive_pid_controller.SetI(pid_struct.Ki);
    _drive_pid_controller.SetD(pid_struct.Kd);
}

void SwerveModule::SetDesiredState(SwerveModuleState state, bool open_loop, bool optimize) {
    Rotation2d encoder_rotation{_GetSteerAngle()};

    // If the wheel needs to rotate over 90 degrees, rotate the other direction and flip the output
    // This prevents the wheel from ever needing to rotate more than 90 degrees
    if (optimize) {
        state = SwerveModuleState::Optimize(state, encoder_rotation);
    }

    // Scale the wheel speed down by the cosine of the angle error
    // This prevents the wheel from accelerating before it has a chance to face the correct direction
    state.speed *= (state.angle - encoder_rotation).Cos();

    // In open loop, treat speed as a percent power
    // In closed loop, try to hit the acutal speed
    if (open_loop) {
        _drive_motor.Set(state.speed / MAX_WHEEL_SPEED);
    } else {
        volt_t drive_output = volt_t{_drive_pid_controller.Calculate(meters_per_second_t{_GetWheelSpeed()}.value(), state.speed.value())};
        volt_t drive_feed_forward = _drive_feed_forward.Calculate(state.speed);
        _drive_motor.SetVoltage(drive_output + drive_feed_forward);
    }

    double steer_output = _steer_pid_controller.Calculate(_GetSteerAngle(), state.angle.Radians());
    _steer_motor.Set(steer_output);
}

SwerveModuleState SwerveModule::GetState() {
    return {_GetWheelSpeed(), _GetSteerAngle()};
}

SwerveModulePosition SwerveModule::GetPosition() {
    return {_GetWheelPosition(), _GetSteerAngle()};
}

feet_per_second_t SwerveModule::_GetWheelSpeed() {
    return WHEEL_RADIUS * radians_per_second_t{_drive_motor.GetVelocity().GetValue() / DRIVE_GEAR_RATIO} / 1_rad;
}

inch_t SwerveModule::_GetWheelPosition() {
    return WHEEL_RADIUS * radian_t{_drive_motor.GetPosition().GetValue() / DRIVE_GEAR_RATIO} / 1_rad;
}

degree_t SwerveModule::_GetSteerAngle() {
    return _steer_encoder.GetAbsolutePosition().GetValue();
}

void SwerveModule::StopMotors() {
    _drive_motor.Set(0);
    _steer_motor.Set(0);
}

void SwerveModule::ResetEncoder() {
    _drive_motor.SetPosition(0_deg);
}

void SwerveModule::SetCoastMode() {
    _drive_motor_config.MotorOutput.NeutralMode = signals::NeutralModeValue::Coast;
    _drive_motor.GetConfigurator().Apply(_drive_motor_config);
}

void SwerveModule::SetBrakeMode() {
    _drive_motor_config.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
    _drive_motor.GetConfigurator().Apply(_drive_motor_config);
}