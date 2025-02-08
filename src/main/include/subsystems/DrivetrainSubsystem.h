// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#ifndef DRIVETRAINSUBSYSTEM_H
#define DRIVETRAINSUBSYSTEM_H

#include "Constants.h"

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <wpi/array.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/drive/DifferentialDrive.h>
#include "ctre/phoenix6/TalonFX.hpp"
#include <frc/RobotController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <ctre/phoenix6/CANcoder.hpp>

class DrivetrainSubsystem : public frc2::SubsystemBase {
    public:
        DrivetrainSubsystem();
        void Periodic() override;

        void Drive(units::meters_per_second_t x_speed, units::meters_per_second_t y_speed, units::radians_per_second_t rotation, bool open_loop=false);
        void SetBrakeMode();
        

        int CheckNotNullModule();

        frc2::CommandPtr PseudoForwardCommand(std::function<double()> fwd);
        frc2::CommandPtr SysIdForwardQuasistatic(frc2::sysid::Direction direction);
        frc2::CommandPtr SysIdForwardDynamic(frc2::sysid::Direction direction);

        frc2::CommandPtr PseudoSidewaysCommand(std::function<double()> fwd);
        frc2::CommandPtr SysIdSidewaysQuasistatic(frc2::sysid::Direction direction);
        frc2::CommandPtr SysIdSidewaysDynamic(frc2::sysid::Direction direction);

    private:
        bool _sideways = false;

        
        units::degree_t _GetSteerAngleFL();
        units::degree_t _GetSteerAngleFR();
        units::degree_t _GetSteerAngleBL();
        units::degree_t _GetSteerAngleBR();

        ctre::phoenix6::hardware::TalonFX _drive_motor_FL{10};
        ctre::phoenix6::hardware::TalonFX _drive_motor_FR{12};
        ctre::phoenix6::hardware::TalonFX _drive_motor_BL{14};
        ctre::phoenix6::hardware::TalonFX _drive_motor_BR{16};

        ctre::phoenix6::hardware::TalonFX _steer_motor_FL{11};
        ctre::phoenix6::hardware::TalonFX _steer_motor_FR{13};
        ctre::phoenix6::hardware::TalonFX _steer_motor_BL{15};
        ctre::phoenix6::hardware::TalonFX _steer_motor_BR{17};

        ctre::phoenix6::hardware::CANcoder _encoder_FL{20};
        ctre::phoenix6::hardware::CANcoder _encoder_FR{21};
        ctre::phoenix6::hardware::CANcoder _encoder_BL{22};
        ctre::phoenix6::hardware::CANcoder _encoder_BR{23};

        ctre::phoenix6::configs::CANcoderConfiguration _encoder_FL_config{};
        ctre::phoenix6::configs::CANcoderConfiguration _encoder_FR_config{};
        ctre::phoenix6::configs::CANcoderConfiguration _encoder_BL_config{};
        ctre::phoenix6::configs::CANcoderConfiguration _encoder_BR_config{};

        frc::DifferentialDrive _drive{[this](auto val) {_drive_motor_FL.Set(val);},
                                       [this](auto val) {_drive_motor_FR.Set(val);}
        };

        frc::ProfiledPIDController<units::radians> _steer_pid_controller_FL{SwerveConstants::DrivetrainConstants::SteerPIDConstants::Kp_Steer, SwerveConstants::DrivetrainConstants::SteerPIDConstants::Ki_Steer, SwerveConstants::DrivetrainConstants::SteerPIDConstants::Kd_Steer, 
            {SwerveConstants::DrivetrainConstants::SteerPIDConstants::MAX_SPEED, SwerveConstants::DrivetrainConstants::SteerPIDConstants::MAX_ACCELERATION}};

        frc::ProfiledPIDController<units::radians> _steer_pid_controller_FR{SwerveConstants::DrivetrainConstants::SteerPIDConstants::Kp_Steer, SwerveConstants::DrivetrainConstants::SteerPIDConstants::Ki_Steer, SwerveConstants::DrivetrainConstants::SteerPIDConstants::Kd_Steer, 
                    {SwerveConstants::DrivetrainConstants::SteerPIDConstants::MAX_SPEED, SwerveConstants::DrivetrainConstants::SteerPIDConstants::MAX_ACCELERATION}};
        
        frc::ProfiledPIDController<units::radians> _steer_pid_controller_BL{SwerveConstants::DrivetrainConstants::SteerPIDConstants::Kp_Steer, SwerveConstants::DrivetrainConstants::SteerPIDConstants::Ki_Steer, SwerveConstants::DrivetrainConstants::SteerPIDConstants::Kd_Steer, 
            {SwerveConstants::DrivetrainConstants::SteerPIDConstants::MAX_SPEED, SwerveConstants::DrivetrainConstants::SteerPIDConstants::MAX_ACCELERATION}};
        
        frc::ProfiledPIDController<units::radians> _steer_pid_controller_BR{SwerveConstants::DrivetrainConstants::SteerPIDConstants::Kp_Steer, SwerveConstants::DrivetrainConstants::SteerPIDConstants::Ki_Steer, SwerveConstants::DrivetrainConstants::SteerPIDConstants::Kd_Steer, 
            {SwerveConstants::DrivetrainConstants::SteerPIDConstants::MAX_SPEED, SwerveConstants::DrivetrainConstants::SteerPIDConstants::MAX_ACCELERATION}};
        
        frc2::sysid::SysIdRoutine _sysIdForwardRoutine{
        frc2::sysid::Config{std::nullopt, std::nullopt, std::nullopt, nullptr},
        frc2::sysid::Mechanism{
            [this](units::volt_t driveVoltage) {
                _drive_motor_FL.SetVoltage(driveVoltage);
                _drive_motor_FR.SetVoltage(driveVoltage);
                // _drive_motor_BL.SetVoltage(driveVoltage);
                // _drive_motor_BR.SetVoltage(driveVoltage);
            },
            [this](frc::sysid::SysIdRoutineLog* log) {
                log->Motor("drive-fl")
                    .voltage(_drive_motor_FL.Get() * frc::RobotController::GetBatteryVoltage());
                    // .position(units::meter_t{2_in * units::radian_t{360_deg * _drive_motor_FL.GetPosition().GetValue() / 2048.0 /(36000.0/5880.0)} / 1_rad})
                    // .velocity(units::meters_per_second_t{2_in * units::radians_per_second_t{_drive_motor_FL.GetVelocity().GetValue()*10.0*360_deg_per_s/2048.0/(36000.0/5880.0)} / 1_rad});
                log->Motor("drive-fr")
                    .voltage(_drive_motor_FR.Get() * frc::RobotController::GetBatteryVoltage());
                    // .position(units::meter_t{2_in * units::radian_t{360_deg * _drive_motor_FR.GetPosition().GetValue() / 2048.0 /(36000.0/5880.0)} / 1_rad})
                    // .velocity(units::meters_per_second_t{2_in * units::radians_per_second_t{_drive_motor_FR.GetVelocity().GetValue()*10.0*360_deg_per_s/2048.0/(36000.0/5880.0)} / 1_rad});
                log->Motor("drive-bl")
                    .voltage(_drive_motor_BL.Get() * frc::RobotController::GetBatteryVoltage());
                    // .position(units::meter_t{2_in * units::radian_t{360_deg * _drive_motor_BL.GetPosition().GetValue() / 2048.0 /(36000.0/5880.0)} / 1_rad})
                    // .velocity(units::meters_per_second_t{2_in * units::radians_per_second_t{_drive_motor_BL.GetVelocity().GetValue()*10.0*360_deg_per_s/2048.0/(36000.0/5880.0)} / 1_rad});
                log->Motor("drive-br")
                    .voltage(_drive_motor_BR.Get() * frc::RobotController::GetBatteryVoltage());
                    // .position(units::meter_t{2_in * units::radian_t{360_deg * _drive_motor_BR.GetPosition().GetValue() / 2048.0 /(36000.0/5880.0)} / 1_rad})
                    // .velocity(units::meters_per_second_t{2_in * units::radians_per_second_t{_drive_motor_BR.GetVelocity().GetValue()*10.0*360_deg_per_s/2048.0/(36000.0/5880.0)} / 1_rad});
            },
            this}};

        frc2::sysid::SysIdRoutine _sysIdSidewaysRoutine{
        frc2::sysid::Config{std::nullopt, std::nullopt, std::nullopt, nullptr},
        frc2::sysid::Mechanism{
            [this](units::volt_t driveVoltage) {
                _drive_motor_FL.SetVoltage(driveVoltage);
                _drive_motor_FR.SetVoltage(driveVoltage);
                // _drive_motor_BL.SetVoltage(driveVoltage);
                // _drive_motor_BR.SetVoltage(driveVoltage);
            },
            [this](frc::sysid::SysIdRoutineLog* log) {
                log->Motor("drive-fl")
                    .voltage(_drive_motor_FL.Get() * frc::RobotController::GetBatteryVoltage());
                    // .position(units::meter_t{2_in * units::radian_t{360_deg * _drive_motor_FL.GetPosition().GetValue() / 2048.0 /(36000.0/5880.0)} / 1_rad})
                    // .velocity(units::meters_per_second_t{2_in * units::radians_per_second_t{_drive_motor_FL.GetVelocity().GetValue()*10.0*360_deg_per_s/2048.0/(36000.0/5880.0)} / 1_rad});
                log->Motor("drive-fr")
                    .voltage(_drive_motor_FR.Get() * frc::RobotController::GetBatteryVoltage());
                    // .position(units::meter_t{2_in * units::radian_t{360_deg * _drive_motor_FR.GetPosition().GetValue() / 2048.0 /(36000.0/5880.0)} / 1_rad})
                    // .velocity(units::meters_per_second_t{2_in * units::radians_per_second_t{_drive_motor_FR.GetVelocity().GetValue()*10.0*360_deg_per_s/2048.0/(36000.0/5880.0)} / 1_rad});
                log->Motor("drive-bl")
                    .voltage(_drive_motor_BL.Get() * frc::RobotController::GetBatteryVoltage());
                    // .position(units::meter_t{2_in * units::radian_t{360_deg * _drive_motor_BL.GetPosition().GetValue() / 2048.0 /(36000.0/5880.0)} / 1_rad})
                    // .velocity(units::meters_per_second_t{2_in * units::radians_per_second_t{_drive_motor_BL.GetVelocity().GetValue()*10.0*360_deg_per_s/2048.0/(36000.0/5880.0)} / 1_rad});
                log->Motor("drive-br")
                    .voltage(_drive_motor_BR.Get() * frc::RobotController::GetBatteryVoltage());
                    // .position(units::meter_t{2_in * units::radian_t{360_deg * _drive_motor_BR.GetPosition().GetValue() / 2048.0 /(36000.0/5880.0)} / 1_rad})
                    // .velocity(units::meters_per_second_t{2_in * units::radians_per_second_t{_drive_motor_BR.GetVelocity().GetValue()*10.0*360_deg_per_s/2048.0/(36000.0/5880.0)} / 1_rad});
            },
            this}};

};


#endif