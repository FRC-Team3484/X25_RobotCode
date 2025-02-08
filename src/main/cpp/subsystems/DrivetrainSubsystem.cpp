// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DrivetrainSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>

//Path Planner Paths
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <frc/DriverStation.h>


using namespace frc;
using namespace units;
using namespace ctre::phoenix6;
using namespace SwerveConstants::DrivetrainConstants;

DrivetrainSubsystem::DrivetrainSubsystem()
{
        SmartDashboard::PutBoolean("Drivetrain Diagnostics", false);
    _sideways = false;
    ctre::phoenix6::configs::TalonFXConfiguration motor_config{};
    motor_config.MotorOutput.Inverted = true;
    motor_config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    
    _drive_motor_FL.GetConfigurator().Apply(motor_config);
    _drive_motor_FR.GetConfigurator().Apply(motor_config);
    _drive_motor_BL.GetConfigurator().Apply(motor_config);
    _drive_motor_BR.GetConfigurator().Apply(motor_config);

    _drive_motor_FL.SetControl(ctre::phoenix6::controls::Follower{_drive_motor_BL.GetDeviceID(), false});
    _drive_motor_FR.SetControl(ctre::phoenix6::controls::Follower{_drive_motor_BR.GetDeviceID(), false});

    _steer_motor_FL.GetConfigurator().Apply(motor_config);
    _steer_motor_FR.GetConfigurator().Apply(motor_config);
    _steer_motor_BL.GetConfigurator().Apply(motor_config);
    _steer_motor_BR.GetConfigurator().Apply(motor_config);

    configs::MagnetSensorConfigs encoder_magnet_FL_config{};
        encoder_magnet_FL_config
            .WithMagnetOffset(SWERVE_FRONT_LEFT.EncoderOffset)
            .WithSensorDirection(false) // not reversed
            .WithAbsoluteSensorDiscontinuityPoint(180_deg);
        
        _encoder_FL_config.MagnetSensor = encoder_magnet_FL_config;
        _encoder_FL.GetConfigurator().Apply(_encoder_FL_config);

    configs::MagnetSensorConfigs encoder_magnet_FR_config{};
        encoder_magnet_FR_config
            .WithMagnetOffset(SWERVE_FRONT_RIGHT.EncoderOffset)
            .WithSensorDirection(false) // not reversed
            .WithAbsoluteSensorDiscontinuityPoint(180_deg);
        
        _encoder_FR_config.MagnetSensor = encoder_magnet_FR_config;
        _encoder_FR.GetConfigurator().Apply(_encoder_FR_config);

    configs::MagnetSensorConfigs encoder_magnet_BL_config{};
        encoder_magnet_BL_config
            .WithMagnetOffset(SWERVE_BACK_LEFT.EncoderOffset)
            .WithSensorDirection(false) // not reversed
            .WithAbsoluteSensorDiscontinuityPoint(180_deg);
        
        _encoder_BL_config.MagnetSensor = encoder_magnet_BL_config;
        _encoder_BL.GetConfigurator().Apply(_encoder_BL_config);

    configs::MagnetSensorConfigs encoder_magnet_BR_config{};
        encoder_magnet_BR_config
            .WithMagnetOffset(SWERVE_BACK_RIGHT.EncoderOffset)
            .WithSensorDirection(false) // not reversed
            .WithAbsoluteSensorDiscontinuityPoint(180_deg);
        
        _encoder_BR_config.MagnetSensor = encoder_magnet_BR_config;
        _encoder_BR.GetConfigurator().Apply(_encoder_BR_config);

    _steer_pid_controller_FL.EnableContinuousInput(-180_deg, 180_deg);
    _steer_pid_controller_FR.EnableContinuousInput(-180_deg, 180_deg);
    _steer_pid_controller_BL.EnableContinuousInput(-180_deg, 180_deg);
    _steer_pid_controller_BR.EnableContinuousInput(-180_deg, 180_deg);
}

void DrivetrainSubsystem::Periodic() {
    if (_sideways) { 
        double steer_output_FR = _steer_pid_controller_FR.Calculate(_GetSteerAngleFR(), 90_deg);
        _steer_motor_FR.Set(steer_output_FR);

        double steer_output_FL = _steer_pid_controller_FL.Calculate(_GetSteerAngleFL(), 90_deg);
        _steer_motor_FL.Set(steer_output_FL);

        double steer_output_BL = _steer_pid_controller_BL.Calculate(_GetSteerAngleBL(), 90_deg);
        _steer_motor_BL.Set(steer_output_BL);

        double steer_output_BR = _steer_pid_controller_BR.Calculate(_GetSteerAngleBR(), 90_deg);
        _steer_motor_BR.Set(steer_output_BR);
    } else if (!_sideways) {
        double steer_output_FR = _steer_pid_controller_FR.Calculate(_GetSteerAngleFR(), 0_deg);
        _steer_motor_FR.Set(steer_output_FR);

        double steer_output_FL = _steer_pid_controller_FL.Calculate(_GetSteerAngleFL(), 0_deg);
        _steer_motor_FL.Set(steer_output_FL);

        double steer_output_BL = _steer_pid_controller_BL.Calculate(_GetSteerAngleBL(), 0_deg);
        _steer_motor_BL.Set(steer_output_BL);

        double steer_output_BR = _steer_pid_controller_BR.Calculate(_GetSteerAngleBR(), 0_deg);
        _steer_motor_BR.Set(steer_output_BR);
    }
}

frc2::CommandPtr DrivetrainSubsystem::PseudoForwardCommand(std::function<double()> fwd) {
    return frc2::cmd::Run([this, fwd] { _sideways = false; _drive.ArcadeDrive(fwd(), 0); }, {this});
}

frc2::CommandPtr DrivetrainSubsystem::SysIdForwardQuasistatic(frc2::sysid::Direction direction) {
    return _sysIdForwardRoutine.Quasistatic(direction);
}

frc2::CommandPtr DrivetrainSubsystem::SysIdForwardDynamic(frc2::sysid::Direction direction) {
    return _sysIdForwardRoutine.Dynamic(direction);
}

frc2::CommandPtr DrivetrainSubsystem::PseudoSidewaysCommand(std::function<double()> fwd) {
    return frc2::cmd::Run([this, fwd] { _sideways = true; _drive.ArcadeDrive(fwd(), 0); }, {this});
}

frc2::CommandPtr DrivetrainSubsystem::SysIdSidewaysQuasistatic(frc2::sysid::Direction direction) {
    return _sysIdSidewaysRoutine.Quasistatic(direction);
}

frc2::CommandPtr DrivetrainSubsystem::SysIdSidewaysDynamic(frc2::sysid::Direction direction) {
    return _sysIdSidewaysRoutine.Dynamic(direction);
}

degree_t DrivetrainSubsystem::_GetSteerAngleFL() {
    return _encoder_FL.GetAbsolutePosition().GetValue();
}

degree_t DrivetrainSubsystem::_GetSteerAngleFR() {
    return _encoder_FL.GetAbsolutePosition().GetValue();
}


degree_t DrivetrainSubsystem::_GetSteerAngleBL() {
    return _encoder_FL.GetAbsolutePosition().GetValue();
}


degree_t DrivetrainSubsystem::_GetSteerAngleBR() {
    return _encoder_FL.GetAbsolutePosition().GetValue();
}

