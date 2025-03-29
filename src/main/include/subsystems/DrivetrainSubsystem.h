#ifndef DRIVETRAINSUBSYSTEM_H
#define DRIVETRAINSUBSYSTEM_H

#include "subsystems/SwerveModule.h"
#include "FRC3484_Lib/components/SC_Photon.h"

#include "Datatypes.h"

#include <studica/AHRS.h>

#include <units/angle.h>
#include <units/angular_velocity.h>

#include <wpi/array.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc2/command/CommandPtr.h>
#include <ctre/phoenix6/Pigeon2.hpp>
#include "OI.h"

class DrivetrainSubsystem : public frc2::SubsystemBase {
    public:
        DrivetrainSubsystem(SC::SC_SwerveConfigs swerve_config_array[4], SC_Photon* vision, int pigeon_id, std::string_view drivetrain_canbus_name, Operator_Interface* oi);
        void Periodic() override;

        void Drive(units::meters_per_second_t x_speed, units::meters_per_second_t y_speed, units::radians_per_second_t rotation, bool open_loop=false);
        void DriveRobotcentric(frc::ChassisSpeeds speeds, bool open_loop=false);
        void DynamicPivotDrive(units::meters_per_second_t x_speed, units::meters_per_second_t y_speed, units::radians_per_second_t rotation, frc::Translation2d center_of_rotation, bool open_loop=false);
        void DynamicPivotDriveRobotcentric(frc::ChassisSpeeds speeds, frc::Translation2d center_of_rotation, bool open_loop=false);
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

        /**
         * Returns a command that will drive the robot to the given pose
         * 
         * @param pose The pose to drive to
         * @return A command that will drive the robot to the given pose
         */
        frc2::CommandPtr GoToPose(frc::Pose2d pose);

        /**
         * Returns the nearest pose to the robot from a vector of poses
         * 
         * @param poses A vector of poses
         * @return The nearest pose from the vector
         */
        frc::Pose2d GetNearestPose(std::vector<frc::Pose2d> poses);

        /**
         * Returns a pose that is offset from the given pose
         * 
         * @param pose The pose to offset
         * @param offset The offset to apply to the pose
         * @return A pose that is offset from the given pose
         */
        frc::Pose2d ApplyOffsetToPose(frc::Pose2d pose, frc::Pose2d offset);

        /**
         * Returns the pose of the reef side with the given letter     
         * 
         * @param letter The letter of the reef side to get the pose of
         * 
         * @return The pose of the reef side 
         */
        frc::Pose2d GetReefSide(std::string letter);

        /**
         * Returns the pose of the closest reef side
         * 
         * @return The pose of the closest reef side
         */
        frc::Pose2d GetClosestReefSide();
        
        /**
         * Returns the pose of the closest feeder station, including which side (offset) is closest (either left or right)
         * 
         * @return The pose of the closest feeder station side
         */
        frc::Pose2d GetClosestFeederStation();

        /**
         * Returns the pose of the closest processor
         * 
         * @return The pose of the closest processor
         */
        frc::Pose2d GetClosestProcessor();

        /**
         * Checks if the robot is at the target position
         * 
         * @return True if the robot is at the target position
         */
        bool GetAtTargetPosition();

        /**
         * Checks if the robot is near the target position
         * 
         * @return True if the robot is near the target position
         */
        bool GetNearTargetPosition();

        
        
        frc::SwerveDriveKinematics<4> kinematics{
            frc::Translation2d{SwerveConstants::DrivetrainConstants::DRIVETRAIN_LENGTH/2, SwerveConstants::DrivetrainConstants::DRIVETRAIN_WIDTH/2},
            frc::Translation2d{SwerveConstants::DrivetrainConstants::DRIVETRAIN_LENGTH/2, -SwerveConstants::DrivetrainConstants::DRIVETRAIN_WIDTH/2},
            frc::Translation2d{-SwerveConstants::DrivetrainConstants::DRIVETRAIN_LENGTH/2, SwerveConstants::DrivetrainConstants::DRIVETRAIN_WIDTH/2},
            frc::Translation2d{-SwerveConstants::DrivetrainConstants::DRIVETRAIN_LENGTH/2, -SwerveConstants::DrivetrainConstants::DRIVETRAIN_WIDTH/2}
        };

    private:
        SwerveModule* _modules[4];

        SC_Photon* _vision;
        
        ctre::phoenix6::hardware::Pigeon2 _pigeon;

        frc::SwerveDrivePoseEstimator<4>* _odometry;

        frc::Field2d _field;

        frc::Pose2d _target_position;
        units::degree_t _pigeon_offset;

        Operator_Interface* _oi;

};

#endif