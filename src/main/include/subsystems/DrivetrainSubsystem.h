// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#ifndef DRIVETRAINSUBSYSTEM_H
#define DRIVETRAINSUBSYSTEM_H

#include "subsystems/SwerveModule.h"
#include "FRC3484_Lib/components/SC_Photon.h"

#include <studica/AHRS.h>

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <wpi/array.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/drive/DifferentialDrive.h>
#include "ctre/phoenix6/TalonFX.hpp"
#include <frc/RobotController.h>

class DrivetrainSubsystem : public frc2::SubsystemBase {
    public:
        DrivetrainSubsystem(SC::SC_SwerveConfigs swerve_config_array[4], SC_Photon* vision);
        void Periodic() override;

        void Drive(units::meters_per_second_t x_speed, units::meters_per_second_t y_speed, units::radians_per_second_t rotation, bool open_loop=false);
        void DriveRobotcentric(frc::ChassisSpeeds speeds, bool open_loop=false);
        void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desired_states, bool open_loop=false, bool optimize=true);
        frc::Rotation2d GetHeading();
        void SetHeading(units::degree_t heading=0_deg);
        units::degrees_per_second_t GetTurnRate();
        frc::Pose2d GetPose();
        void ResetOdometry(frc::Pose2d pose);
        wpi::array<frc::SwerveModulePosition, 4> GetModulePositions();
        frc::ChassisSpeeds GetChassisSpeeds();
        void StopMotors();
        void ResetEncoders();
        void SetCoastMode();
        void SetBrakeMode();
        frc::Rotation2d GetHeadingAuto();
        void ResetOdometryAuto(frc::Pose2d pose);

        int CheckNotNullModule();

        frc::SwerveDriveKinematics<4> kinematics{
            frc::Translation2d{SwerveConstants::DrivetrainConstants::DRIVETRAIN_LENGTH/2, SwerveConstants::DrivetrainConstants::DRIVETRAIN_WIDTH/2},
            frc::Translation2d{SwerveConstants::DrivetrainConstants::DRIVETRAIN_LENGTH/2, -SwerveConstants::DrivetrainConstants::DRIVETRAIN_WIDTH/2},
            frc::Translation2d{-SwerveConstants::DrivetrainConstants::DRIVETRAIN_LENGTH/2, SwerveConstants::DrivetrainConstants::DRIVETRAIN_WIDTH/2},
            frc::Translation2d{-SwerveConstants::DrivetrainConstants::DRIVETRAIN_LENGTH/2, -SwerveConstants::DrivetrainConstants::DRIVETRAIN_WIDTH/2}
        };

        frc2::CommandPtr PseudoForwardCommand(std::function<double()> fwd);
        frc2::CommandPtr SysIdForwardQuasistatic(frc2::sysid::Direction direction);
        frc2::CommandPtr SysIdForwardDynamic(frc2::sysid::Direction direction);

        frc2::CommandPtr PseudoSidewaysCommand(std::function<double()> fwd);
        frc2::CommandPtr SysIdSidewaysQuasistatic(frc2::sysid::Direction direction);
        frc2::CommandPtr SysIdSidewaysDynamic(frc2::sysid::Direction direction);

    private:
        SwerveModule* _modules[4];
            
        ctre::phoenix6::hardware::TalonFX _drive_motor_FL{10};
        ctre::phoenix6::hardware::TalonFX _drive_motor_FR{12};
        ctre::phoenix6::hardware::TalonFX _drive_motor_BL{14};
        ctre::phoenix6::hardware::TalonFX _drive_motor_BR{16};

        ctre::phoenix6::hardware::TalonFX _steer_motor_FL{11};
        ctre::phoenix6::hardware::TalonFX _steer_motor_FR{13};
        ctre::phoenix6::hardware::TalonFX _steer_motor_BL{15};
        ctre::phoenix6::hardware::TalonFX _steer_motor_BR{17};

        studica::AHRS* _gyro;
        units::degree_t _gyro_offset = 0_deg;

        frc::SwerveDriveOdometry<4>* _odometry;

        frc::Field2d _field;

        SC_Photon* _vision;
        frc::DifferentialDrive _drive{[this](auto val) {_drive_motor_FL.Set(val);},
                                       [this](auto val) {_drive_motor_FR.Set(val);}
        };

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