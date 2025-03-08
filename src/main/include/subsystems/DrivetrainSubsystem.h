#ifndef DRIVETRAINSUBSYSTEM_H
#define DRIVETRAINSUBSYSTEM_H

#include "subsystems/SwerveModule.h"
#include "FRC3484_Lib/components/SC_Photon.h"

#include <frc/RobotController.h>

#include "Datatypes.h"

#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc2/command/Commands.h>
#include "Constants.h"

#include <units/angle.h>
#include <units/angular_velocity.h>

#include <wpi/array.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc2/command/CommandPtr.h>
#include <ctre/phoenix6/Pigeon2.hpp>

class DrivetrainSubsystem: public frc2::SubsystemBase {
    public:
        DrivetrainSubsystem(SC::SC_SwerveConfigs swerve_config_array[4], SC_Photon* vision, int pigeon_id, std::string_view drivetrain_canbus_name);
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
         * Returns the pose of the closest reef side
         * 
         * @param reef_offset The left or the right side of the reef to align to
         * @return The pose of the closest reef side
         */
        frc::Pose2d GetClosestReefSide(ReefAlignment reef_offset);
        
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

        frc2::CommandPtr PseudoDriveCommand(std::function<double()> fwd,
                                      std::function<double()> side,
                                      std::function<double()> rot,
                                      std::function<bool()> reset_gyro);
        frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction, units::degree_t sysID_direction);
        frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction, units::degree_t sysID_direction);
        
        
        frc::SwerveDriveKinematics<4> kinematics{
            frc::Translation2d{SwerveConstants::DrivetrainConstants::DRIVETRAIN_LENGTH/2, SwerveConstants::DrivetrainConstants::DRIVETRAIN_WIDTH/2},
            frc::Translation2d{SwerveConstants::DrivetrainConstants::DRIVETRAIN_LENGTH/2, -SwerveConstants::DrivetrainConstants::DRIVETRAIN_WIDTH/2},
            frc::Translation2d{-SwerveConstants::DrivetrainConstants::DRIVETRAIN_LENGTH/2, SwerveConstants::DrivetrainConstants::DRIVETRAIN_WIDTH/2},
            frc::Translation2d{-SwerveConstants::DrivetrainConstants::DRIVETRAIN_LENGTH/2, -SwerveConstants::DrivetrainConstants::DRIVETRAIN_WIDTH/2}
        };

    private:
        SwerveModule* _modules[4];

        units::degree_t _sysID_direction = 0_deg;

        SC_Photon* _vision;
            
        ctre::phoenix6::hardware::Pigeon2 _pigeon;

        frc::SwerveDriveOdometry<4>* _odometry;

        frc::Field2d _field;

        frc::Pose2d _target_position;

        
        frc2::sysid::SysIdRoutine _sysIdRoutine{
            frc2::sysid::Config{std::nullopt, std::nullopt, std::nullopt,
                                nullptr},
            frc2::sysid::Mechanism{
                [this](units::volt_t driveVoltage) {
                _modules[FL]->SetVoltage(driveVoltage, _sysID_direction);
                _modules[FR]->SetVoltage(driveVoltage, _sysID_direction);
                _modules[BL]->SetVoltage(driveVoltage, _sysID_direction);
                _modules[BR]->SetVoltage(driveVoltage, _sysID_direction);
                
                },
                [this](frc::sysid::SysIdRoutineLog* log) {
                log->Motor("drive-fl")
                    .voltage(_modules[FL]->Get() *
                                frc::RobotController::GetBatteryVoltage())
                    .position(_modules[FL]->GetPosition().distance)
                    .velocity(_modules[FL]->GetState().speed);
                log->Motor("drive-fr")
                    .voltage(_modules[FR]->Get() *
                                frc::RobotController::GetBatteryVoltage())
                    .position(_modules[FR]->GetPosition().distance)
                    .velocity(_modules[FR]->GetState().speed);
                log->Motor("drive-bl")
                    .voltage(_modules[BL]->Get() *
                                frc::RobotController::GetBatteryVoltage())
                    .position(_modules[BL]->GetPosition().distance)
                    .velocity(_modules[BL]->GetState().speed);
                log->Motor("drive-br")
                    .voltage(_modules[BR]->Get() *
                                frc::RobotController::GetBatteryVoltage())
                    .position(_modules[BR]->GetPosition().distance)
                    .velocity(_modules[BR]->GetState().speed);
                },
                this}};
};

#endif