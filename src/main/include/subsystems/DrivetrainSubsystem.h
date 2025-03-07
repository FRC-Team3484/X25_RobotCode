#ifndef DRIVETRAINSUBSYSTEM_H
#define DRIVETRAINSUBSYSTEM_H

#include <frc/drive/DifferentialDrive.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANCoder.hpp>
#include <frc/RobotController.h>

#include "Constants.h"


#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc2/command/Commands.h>

#include "Datatypes.h"

#include <studica/AHRS.h>

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

class DrivetrainSubsystem : public frc2::SubsystemBase {
    public:
        DrivetrainSubsystem();
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

        void ResetEncoder();

        int CheckNotNullModule();

        frc2::CommandPtr PseudoDriveCommand(std::function<double()> fwd,
                                      std::function<double()> rot);
        frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction);
        frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction);

        units::degree_t GetSteerAngleFL();
        units::degree_t GetSteerAngleFR();
        units::degree_t GetSteerAngleBL();
        units::degree_t GetSteerAngleBR();

    private:

        


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

        ctre::phoenix6::configs::TalonFXConfiguration _drive_motor_config{};
        ctre::phoenix6::configs::TalonFXConfiguration _steer_motor_config{};
        ctre::phoenix6::configs::CANcoderConfiguration _encoder_config{};

        // PID Loops

        // frc::PIDController _drive_pid_controller{1.0,0.0,0.0};

        SC::SC_SwerveCurrents _swerve_current_constants;

        frc::DifferentialDrive _drive{[this](auto val) { _drive_motor_FL.Set(val); },
                                    [this](auto val) { _drive_motor_FR.Set(val); }
        };


        frc2::sysid::SysIdRoutine _sysIdRoutine{
            frc2::sysid::Config{std::nullopt, std::nullopt, std::nullopt,
                                nullptr},
            frc2::sysid::Mechanism{
                [this](units::volt_t driveVoltage) {
                _drive_motor_FL.SetVoltage(driveVoltage);
                _drive_motor_FR.SetVoltage(driveVoltage);
                // _drive_motor_BL.SetVoltage(driveVoltage);
                // _drive_motor_BR.SetVoltage(driveVoltage);
                },
                [this](frc::sysid::SysIdRoutineLog* log) {
                log->Motor("drive-fl")
                    .voltage(_drive_motor_FL.Get() *
                                frc::RobotController::GetBatteryVoltage())
                    .position(units::meter_t(SwerveConstants::DrivetrainConstants::WHEEL_RADIUS * (units::angle::radian_t(_drive_motor_FL.GetPosition().GetValue()) / SwerveConstants::DrivetrainConstants::DRIVE_GEAR_RATIO)))
                    .velocity(units::meters_per_second_t(SwerveConstants::DrivetrainConstants::WHEEL_RADIUS * (units::angular_velocity::radians_per_second_t(_drive_motor_BR.GetVelocity().GetValue()) / SwerveConstants::DrivetrainConstants::DRIVE_GEAR_RATIO)));
                log->Motor("drive-fr")
                    .voltage(_drive_motor_FR.Get() *
                                frc::RobotController::GetBatteryVoltage())
                    .position(units::meter_t(SwerveConstants::DrivetrainConstants::WHEEL_RADIUS * (units::angle::radian_t(_drive_motor_FL.GetPosition().GetValue()) / SwerveConstants::DrivetrainConstants::DRIVE_GEAR_RATIO)))
                    .velocity(units::meters_per_second_t(SwerveConstants::DrivetrainConstants::WHEEL_RADIUS * (units::angular_velocity::radians_per_second_t(_drive_motor_BR.GetVelocity().GetValue()) / SwerveConstants::DrivetrainConstants::DRIVE_GEAR_RATIO)));
                log->Motor("drive-bl")
                    .voltage(_drive_motor_BL.Get() *
                                frc::RobotController::GetBatteryVoltage())
                    .position(units::meter_t(SwerveConstants::DrivetrainConstants::WHEEL_RADIUS * (units::angle::radian_t(_drive_motor_FL.GetPosition().GetValue()) / SwerveConstants::DrivetrainConstants::DRIVE_GEAR_RATIO)))
                    .velocity(units::meters_per_second_t(SwerveConstants::DrivetrainConstants::WHEEL_RADIUS * (units::angular_velocity::radians_per_second_t(_drive_motor_BR.GetVelocity().GetValue()) / SwerveConstants::DrivetrainConstants::DRIVE_GEAR_RATIO)));
                log->Motor("drive-br")
                    .voltage(_drive_motor_BR.Get() * frc::RobotController::GetBatteryVoltage())
                    .position(units::meter_t(SwerveConstants::DrivetrainConstants::WHEEL_RADIUS * (units::angle::radian_t(_drive_motor_FL.GetPosition().GetValue()) / SwerveConstants::DrivetrainConstants::DRIVE_GEAR_RATIO)))
                    .velocity(units::meters_per_second_t(SwerveConstants::DrivetrainConstants::WHEEL_RADIUS * (units::angular_velocity::radians_per_second_t(_drive_motor_BR.GetVelocity().GetValue()) / SwerveConstants::DrivetrainConstants::DRIVE_GEAR_RATIO)));
                },
                this}};

            
};

#endif