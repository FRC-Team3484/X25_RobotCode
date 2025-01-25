#ifndef SWERVE_MODULE_H
#define SWERVE_MODULE_H

#include "Constants.h"

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>

class SwerveModule {
    public:
        SwerveModule(SC::SC_SwerveConfigs corner, SC::SC_SwervePID pid_struct);
        void SetDesiredState(frc::SwerveModuleState state, bool open_loop, bool optimize=true);
        frc::SwerveModuleState GetState();
        frc::SwerveModulePosition GetPosition();
        void StopMotors();
        void ResetEncoder();
        void SetCoastMode();
        void SetBrakeMode();

    private:
        ctre::phoenix6::hardware::TalonFX _drive_motor;
        ctre::phoenix6::hardware::TalonFX _steer_motor;
        ctre::phoenix6::hardware::CANcoder _steer_encoder;

        ctre::phoenix6::configs::TalonFXConfiguration _drive_motor_config{};
        ctre::phoenix6::configs::TalonFXConfiguration _steer_motor_config{};
        ctre::phoenix6::configs::CANcoderConfiguration _encoder_config{};

        frc::PIDController _drive_pid_controller{0, 0, 0};
        
        frc::ProfiledPIDController<units::radians> _steer_pid_controller{SwerveConstants::DrivetrainConstants::SteerPIDConstants::Kp_Steer, SwerveConstants::DrivetrainConstants::SteerPIDConstants::Ki_Steer, SwerveConstants::DrivetrainConstants::SteerPIDConstants::Kd_Steer, 
            {SwerveConstants::DrivetrainConstants::SteerPIDConstants::MAX_SPEED, SwerveConstants::DrivetrainConstants::SteerPIDConstants::MAX_ACCELERATION}};

        units::feet_per_second_t _GetWheelSpeed();
        units::inch_t _GetWheelPosition();
        units::degree_t _GetSteerAngle();

        frc::SimpleMotorFeedforward<units::meters> _drive_feed_forward;
};

#endif