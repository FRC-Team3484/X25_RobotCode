#include "Robot.h"

#include <frc2/command/CommandScheduler.h>

Robot::Robot() {}

void Robot::RobotPeriodic() {
    frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
    _drive_state_commands.Schedule();
    _driver_robot_state = drive;
    _operator_drive_robot_state = stow;
}

void Robot::TeleopPeriodic() {
    switch (_driver_robot_state) {
        case drive:
            _test_state_commands.Cancel();
            OperatorPeriodic();
            if (_oi_driver.GetCoralPickup()) {
                _driver_robot_state = auto_pickup_coral;
                _drive_state_commands.Cancel();
                _drive_state_commands.Schedule();
            } else if (_oi_driver.GetAlgaePickup()) {
                _driver_robot_state = auto_pickup_algae;
                _drive_state_commands.Cancel();
                _drive_state_commands.Schedule();
            } else if (_oi_driver.GetScoreReef()) {
                _driver_robot_state = auto_score_reef;
                _drive_state_commands.Cancel();
                _drive_state_commands.Schedule();
            } else if (_oi_driver.GetScoreProcessor()) {
                _driver_robot_state = auto_score_processor;
                _drive_state_commands.Cancel();
                _drive_state_commands.Schedule();
            }
            break;
        case auto_pickup_coral:
            
            
            if (!_oi_driver.GetCoralPickup())StartDriveState();
            break;
        case auto_pickup_algae:
            

            if (!_oi_driver.GetAlgaePickup())StartDriveState();
            break;
        case auto_score_reef:
            

            if (!_oi_driver.GetScoreReef())StartDriveState();
            break;
        case auto_score_processor:
            

            if (!_oi_driver.GetScoreProcessor())StartDriveState();
            break;
        default:
            _driver_robot_state = drive;
    }
}

void Robot::OperatorPeriodic() {
    switch (_operator_drive_robot_state){
        case stow:
            _stow_state_commands.Schedule();
            _intake_algae_commands.Cancel();
            _intake_coral_commands.Cancel();
            _processor_commands.Cancel();
            _score_algae_commands.Cancel();
            _score_coral_commands.Cancel();
            break;
        case manual_score_coral:

            break;
        case manual_score_algae:
        
            break;
        case manual_score_processor:
            break;
        case manual_remove_algae:
            break;
        case manual_remove_coral:
            break;
        case ground_pickup:
            break;
        case score_net:
            break;
        case climb:
            break;
        default:
            _operator_drive_robot_state = stow;
    }
}

void Robot::StartDriveState() {
        _driver_robot_state = drive;
        _drive_state_commands.Cancel();
        _drive_state_commands.Schedule();
}

void Robot::StartOperatorState() {
        _operator_drive_robot_state = stow;
        _stow_state_commands.Cancel();
        _stow_state_commands.Schedule();
        _intake_algae_commands.Cancel();
        _intake_coral_commands.Cancel();
        _processor_commands.Cancel();
        _score_algae_commands.Cancel();
        _score_coral_commands.Cancel();
}

void Robot::TeleopExit() {}

void Robot::TestInit() {
    frc2::CommandScheduler::GetInstance().CancelAll();
    _test_state_commands.Schedule();
    StartDriveState();
    StartOperatorState();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif
