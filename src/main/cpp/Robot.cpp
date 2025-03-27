#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <frc/Filesystem.h>
#include <wpinet/WebServer.h>

Robot::Robot() {
    wpi::WebServer::GetInstance().Start(5800, frc::filesystem::GetDeployDirectory());

    _auton_start_positions.AddOption("A", SwerveConstants::AutonDriveConstants::STARTING_POSITION_A);
    _auton_start_positions.AddOption("B", SwerveConstants::AutonDriveConstants::STARTING_POSITION_B);
    _auton_start_positions.AddOption("C", SwerveConstants::AutonDriveConstants::STARTING_POSITION_C);

    _basic_autons.SetDefaultOption("None", "none");
    _basic_autons.AddOption("Drive Forwards", "drive");
    _basic_autons.AddOption("Drive and Score", "score");

    frc::SmartDashboard::PutData("Auton Start Position", &_auton_start_positions);
    frc::SmartDashboard::PutData("Basic Autons", &_basic_autons);
}

void Robot::RobotPeriodic() {
    frc2::CommandScheduler::GetInstance().Run();

    frc::SmartDashboard::PutNumber("Voltage", frc::DriverStation::GetBatteryVoltage());
    
    _match_time = frc::DriverStation::GetMatchTime();
    frc::SmartDashboard::PutNumber("Match Time", _match_time.to<double>());
}

void Robot::DisabledInit() {
    frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::DisabledPeriodic() {
    #ifdef LED_ENABLED
    if (_pdp.GetVoltage() < 12.2) {
        _low_battery = true;
    }
    if (_has_been_enabled) {
        _leds->WaveAnimation();
    } else {
        if (_low_battery){
            _leds->LowBatteryAnimation();
        } else
            _leds->TetrisAnimation();
    }
    #endif
}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
    // _drivetrain->ResetOdometry(_auton_start_positions.GetSelected());

    _has_been_enabled = true;

    _auton_command = _auton_generator->GetAutonomousCommand(_basic_autons.GetSelected());
    _drivetrain->SetBrakeMode();

    if (_auton_command) {
        _auton_command->Schedule();
    }
}

void Robot::AutonomousPeriodic() {
    #ifdef LED_ENABLED
    _leds->TetrisAnimation();
    #endif
}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
    StartDriveState();
    _has_been_enabled = true;
    StartOperatorState();
}

void Robot::TeleopPeriodic() {
    OperatorPeriodic();
    #ifdef LED_ENABLED
    LedPeriodic();
    #endif

    if (_oi_operator->GetReset()) {
        ResetAllSubsystems();
    }

    switch (_driver_robot_state) {
        case drive:
            if (_oi_driver->GetCoralPickup()) {
                _driver_robot_state = auto_pickup_coral_driver;
                CancelDriverCommands();

                _go_to_pose_command = _drivetrain->GoToPose(_drivetrain->GetClosestFeederStation());
                _go_to_pose_command.Schedule();

            } else if ((_oi_driver->GetScoreReef() || _oi_driver->GetAlgaePickup())) {
                _driver_robot_state = auto_reef_driver;
                CancelDriverCommands();

                _go_to_pose_command = _drivetrain->GoToPose(_drivetrain->GetClosestReefSide());
                _go_to_pose_command.Schedule();

            } else if (_oi_driver->GetScoreProcessor()) {
                _driver_robot_state = auto_score_processor_driver;
                CancelDriverCommands();

                _go_to_pose_command = _drivetrain->GoToPose(_drivetrain->GetClosestProcessor());
                _go_to_pose_command.Schedule();
            }
            break;

        //auto
        case auto_pickup_coral_driver:
            
            if (!_oi_driver->GetCoralPickup()) {CancelDriverCommands(); StartDriveState(); }
            break;

        case auto_reef_driver:
            
            if (!(_oi_driver->GetScoreReef() || _oi_driver->GetAlgaePickup())) { CancelDriverCommands(); StartDriveState(); }
            break;
        case auto_score_processor_driver:
            
            if (!_oi_driver->GetScoreProcessor()) { CancelDriverCommands(); StartDriveState(); }
            break;
        default:
            _driver_robot_state = drive;
    }
}

void Robot::OperatorPeriodic() {
    switch (_operator_drive_robot_state)
    {
        case stow:
            if (GetLoadCoralCondition()) {
                CancelOperatorCommands();
                _intake_coral_commands.Schedule();

                _operator_drive_robot_state = pickup_coral;
            } else if (GetScoreReefCondition()) {
                CancelOperatorCommands();
                _score_coral_commands.Schedule();

                _operator_drive_robot_state = score_coral;
            } else if (GetRemoveAlgaeCondition()) {
                CancelOperatorCommands();
                _score_coral_commands.Schedule();

                _operator_drive_robot_state = remove_algae;
            }
            break;
        case pickup_coral:
            if (!GetLoadCoralCondition()) {CancelOperatorCommands(); StartOperatorState();}
            break;
        case score_coral:
            if (!GetScoreReefCondition()) {CancelOperatorCommands(); StartOperatorState();}
            break;
        case remove_algae:
            if (!GetRemoveAlgaeCondition()) {CancelOperatorCommands(); StartOperatorState();}
            break;
            
        default:
            _operator_drive_robot_state = stow;
    }
}

void Robot::LedPeriodic() {
    #ifdef INTAKE_ENABLED
    if (_intake->HasCoral())_leds->HasCoralAnimation(); 
    else if (_intake->HasAlgae()) _leds->HasAlgaeAnimation(); 
    else if (_oi_driver->GetDynamicPivot()) _leds->PivotAnimation(); 
    else if (_driver_robot_state == drive) _leds->DrivingAnimation();
    else if (_driver_robot_state == drive && _operator_drive_robot_state == stow) _leds->ElevatorHomingAnimation();
    else if (_driver_robot_state == auto_score_processor_driver || _driver_robot_state == auto_reef_driver || _driver_robot_state == auto_pickup_coral_driver) {
        _leds->PathAnimation();
    } else if (_operator_drive_robot_state == score_coral) {
        _leds->ScoringLevelFourAnimation();
    }
    #else
    if (_oi_driver->GetDynamicPivot()) _leds->PivotAnimation(); 
    else if (_driver_robot_state == drive) _leds->DrivingAnimation();
    else if (_operator_drive_robot_state == stow) _leds->ElevatorHomingAnimation();
    else if (_driver_robot_state == auto_score_processor_driver || _driver_robot_state == auto_reef_driver || _driver_robot_state == auto_pickup_coral_driver) {
        _leds->PathAnimation();
    } else if (_operator_drive_robot_state == score_coral) {
        _leds->ScoringAnimation();
    }
    #endif
    
}

// This functiion is called to enter the drive state in teleop (the default state)
void Robot::StartDriveState() {
    _driver_robot_state = drive;
    if(!_drive_state_commands.IsScheduled())
        _drive_state_commands.Schedule();
    
    if (_go_to_pose_command.IsScheduled())
        _go_to_pose_command.Cancel();
}

// This functiion is called to enter the stow state in teleop (the default state)
void Robot::StartOperatorState() {
    _operator_drive_robot_state = stow;
    _stow_state_commands.Schedule();
}

void Robot::CancelDriverCommands() {
    if (_drive_state_commands.IsScheduled()) _drive_state_commands.Cancel();
}

void Robot::CancelOperatorCommands() {
    _stow_state_commands.Cancel();
    _intake_algae_commands.Cancel();
    _intake_coral_commands.Cancel();
    _processor_commands.Cancel();
    _score_coral_commands.Cancel();
    _climb_up_state_commands.Cancel();
}
bool Robot::GetLoadCoralCondition(){return _oi_operator->GetLoadCoral()  && (!_intake->CoralLow());}
bool Robot::GetRemoveAlgaeCondition(){return ((_oi_operator->GetReefLevel() == 2 || _oi_operator->GetReefLevel() == 3) && !_oi_operator->GetAlgaeOrCoral());}
bool Robot::GetScoreReefCondition(){return ((!(_oi_operator->GetReefLevel() == 0) && (_oi_operator->GetAlgaeOrCoral())) );}

void Robot::ResetAllSubsystems() {
    frc2::CommandScheduler::GetInstance().CancelAll();

    _pivot->SetStateToHome();
    _elevator->SetStateToHome();

    StartDriveState();
    StartOperatorState();
}

void Robot::TeleopExit() {
    CancelDriverCommands();
    CancelOperatorCommands();
}

void Robot::TestInit() {
    frc2::CommandScheduler::GetInstance().CancelAll();
    _test_state_commands.Schedule();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {
    _test_state_commands.Cancel();
}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif
