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
                CancelDriverStates();
                
                _drive_to_feeder_station = _drivetrain.GoToPose(_drivetrain.GetClosestFeederStation());
                _drive_to_feeder_station.Schedule();

            } else if (_oi_driver.GetAlgaePickup()) {
                _driver_robot_state = auto_pickup_algae;
                CancelDriverStates();

                _drive_to_reef = _drivetrain.GoToPose(_drivetrain.GetClosestReefSide(ReefAlignment::center));
                _drive_to_reef.Schedule();

            } else if (_oi_driver.GetScoreReef()) {
                _driver_robot_state = auto_score_reef;
                CancelDriverStates();

                _drive_to_reef = _drivetrain.GoToPose(_drivetrain.GetClosestReefSide(_oi_operator.GetReefAlignment()));
                _drive_to_reef.Schedule();
                
            } else if (_oi_driver.GetScoreProcessor()) {
                _driver_robot_state = auto_score_processor;
                CancelDriverStates();

                _drive_to_processor = _drivetrain.GoToPose(_drivetrain.GetClosestProcessor());
                _drive_to_processor.Schedule();
            }
            break;
        case auto_pickup_coral:
            if (_oi_operator.GetLoadCoral()) {
                _intake_coral_commands.Schedule();
            }
            
            if (!_oi_driver.GetCoralPickup()) { _intake_coral_commands.Cancel(); StartDriveState(); }
            break;
        case auto_pickup_algae:
            if ((_oi_operator.GetReefLevel() == 2 || _oi_operator.GetReefLevel() == 3) && _oi_operator.GetReefAlignment() == ReefAlignment::center) {
                _intake_algae_commands.Schedule();
            }

            if (!_oi_driver.GetAlgaePickup()) { _intake_algae_commands.Cancel(); StartDriveState(); }
            break;
        case auto_score_reef:
            if (!(_oi_operator.GetReefLevel() == 0) && _oi_operator.GetReefAlignment() == ReefAlignment::left || _oi_operator.GetReefAlignment() == ReefAlignment::right) {
                _score_coral_commands.Schedule();
            }
            
            if (!_oi_driver.GetScoreReef()) { _score_coral_commands.Cancel(); StartDriveState(); }
            break;
        case auto_score_processor:
            if (_oi_operator.GetProcessor()) {
                _processor_commands.Schedule();
            }
            
            if (!_oi_driver.GetScoreProcessor()) { _processor_commands.Cancel(); StartDriveState(); }
            break;
        default:
            _driver_robot_state = drive;
    }
}

void Robot::OperatorPeriodic() {
    switch (_operator_drive_robot_state){
        case stow:
            if (_oi_operator.GetReefLevel() != 0) {
                _operator_drive_robot_state = manual_score_coral;
                CancelOperatorStates();
                
                _score_coral_commands.Schedule();
            } else if (_oi_operator.GetProcessor()) {
                CancelOperatorStates();
                _operator_drive_robot_state = manual_score_processor;
            } else if ((_oi_operator.GetReefLevel() == 2 || _oi_operator.GetReefLevel() == 3) && _oi_operator.GetReefAlignment() == ReefAlignment::center) {
                _operator_drive_robot_state = manual_remove_algae;
                CancelOperatorStates();
                
                _score_coral_commands.Schedule();
            } else if (_oi_operator.GetClimbUp()) {
                _operator_drive_robot_state = climb;
                CancelOperatorStates();
                
                // TODO: Add climb command
            }
            
            break;
        case manual_score_coral:
            break;
        case manual_score_processor:
            break;
        case manual_remove_algae:
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

void Robot::CancelDriverStates() {
    _drive_state_commands.Cancel();
    _drive_to_feeder_station.Cancel();
    _drive_to_reef.Cancel();
    _drive_to_processor.Cancel();
}

void Robot::CancelOperatorStates() {
    _stow_state_commands.Cancel();
    _intake_algae_commands.Cancel();
    _intake_coral_commands.Cancel();
    _processor_commands.Cancel();
    _score_coral_commands.Cancel();
}
    
void Robot::StartOperatorState() {
    _operator_drive_robot_state = stow;
    _stow_state_commands.Cancel();
    _stow_state_commands.Schedule();
    
    _intake_algae_commands.Cancel();
    _intake_coral_commands.Cancel();
    _processor_commands.Cancel();
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
