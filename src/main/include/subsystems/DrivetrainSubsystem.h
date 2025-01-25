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

#include <wpi/array.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/smartdashboard/Field2d.h>

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

    private:
        SwerveModule* _modules[4];
            
        studica::AHRS* _gyro;
        units::degree_t _gyro_offset = 0_deg;

        frc::SwerveDriveOdometry<4>* _odometry;

        frc::Field2d _field;

        SC_Photon* _vision;
};

#endif