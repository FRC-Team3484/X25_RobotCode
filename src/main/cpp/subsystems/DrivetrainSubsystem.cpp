#include "subsystems/DrivetrainSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>

//Path Planner Paths
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <frc/DriverStation.h>

using namespace SC;
using namespace SwerveConstants::DrivetrainConstants;
using namespace SwerveConstants::AutonDriveConstants;
using namespace VisionConstants;

using namespace frc;
using namespace units;
using namespace pathplanner;

DrivetrainSubsystem::DrivetrainSubsystem()
{
        
    ctre::phoenix6::configs::CurrentLimitsConfigs drive_current_limit{};
        drive_current_limit
            .WithSupplyCurrentLimitEnable(_swerve_current_constants.Current_Limit_Enable)
            .WithSupplyCurrentLimit(_swerve_current_constants.Current_Limit_Drive)
            .WithSupplyCurrentLowerLimit(_swerve_current_constants.Drive_Current_Threshold)
            .WithSupplyCurrentLowerTime(_swerve_current_constants.Drive_Current_Time);

        _drive_motor_config.CurrentLimits = drive_current_limit;
        _drive_motor_config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25_s;
        _drive_motor_FL.GetConfigurator().Apply(_drive_motor_config);
        _drive_motor_FR.GetConfigurator().Apply(_drive_motor_config);
        _drive_motor_BL.GetConfigurator().Apply(_drive_motor_config);
        _drive_motor_BR.GetConfigurator().Apply(_drive_motor_config);
        SetBrakeMode();
        ResetEncoder();

        // Create and apply steer configs
        ctre::phoenix6::configs::CurrentLimitsConfigs steer_current_limit{};
        steer_current_limit
            .WithSupplyCurrentLimitEnable(_swerve_current_constants.Current_Limit_Enable)
            .WithSupplyCurrentLimit(_swerve_current_constants.Current_Limit_Steer)
            .WithSupplyCurrentLowerLimit(_swerve_current_constants.Steer_Current_Threshold)
            .WithSupplyCurrentLowerTime(_swerve_current_constants.Steer_Current_Time);


        _steer_motor_config.CurrentLimits = steer_current_limit;
        _steer_motor_config.MotorOutput.Inverted = _swerve_current_constants.Steer_Motor_Reversed;
        _steer_motor_config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;

        _steer_motor_FL.GetConfigurator().Apply(_steer_motor_config);
        _steer_motor_FR.GetConfigurator().Apply(_steer_motor_config);
        _steer_motor_BL.GetConfigurator().Apply(_steer_motor_config);
        _steer_motor_BR.GetConfigurator().Apply(_steer_motor_config);
}

void DrivetrainSubsystem::Periodic() {
    double steer_output_FR = _steer_pid_controller_FR.Calculate(GetSteerAngleFR(), radian_t{0_deg});
    _steer_motor_FR.Set(steer_output_FR);

    double steer_output_FL = _steer_pid_controller_FL.Calculate(GetSteerAngleFL(), radian_t{0_deg});
    _steer_motor_FL.Set(steer_output_FL);

    double steer_output_BL = _steer_pid_controller_BL.Calculate(GetSteerAngleBL(), radian_t{0_deg});
    _steer_motor_BL.Set(steer_output_BL);

    double steer_output_BR = _steer_pid_controller_BR.Calculate(GetSteerAngleBR(), radian_t{0_deg});
    _steer_motor_BR.Set(steer_output_BR);
}

frc2::CommandPtr DrivetrainSubsystem::PseudoDriveCommand(std::function<double()> fwd,
                                           std::function<double()> rot) {
  	return frc2::cmd::Run([this, fwd, rot] { _drive.ArcadeDrive(fwd(), rot()); },
                        {this})
      	.WithName("Psuedo Testing Arcade Drive");
}

frc2::CommandPtr DrivetrainSubsystem::SysIdQuasistatic(frc2::sysid::Direction direction) {
  	return _sysIdRoutine.Quasistatic(direction);
}

frc2::CommandPtr DrivetrainSubsystem::SysIdDynamic(frc2::sysid::Direction direction) {
  	return _sysIdRoutine.Dynamic(direction);
}

void DrivetrainSubsystem::SetBrakeMode() {
    _drive_motor_config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    _drive_motor_FL.GetConfigurator().Apply(_drive_motor_config);
    _drive_motor_FR.GetConfigurator().Apply(_drive_motor_config);
    _drive_motor_BL.GetConfigurator().Apply(_drive_motor_config);
    _drive_motor_BR.GetConfigurator().Apply(_drive_motor_config);
}

void DrivetrainSubsystem::ResetEncoder() {
    _drive_motor_FL.SetPosition(0_deg);
    _drive_motor_FR.SetPosition(0_deg);
    _drive_motor_BL.SetPosition(0_deg);
    _drive_motor_BR.SetPosition(0_deg);
}