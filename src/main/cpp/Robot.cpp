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
    StartDriveState();
    StartOperatorState();
}

void Robot::TeleopPeriodic() {
    switch (_driver_robot_state) {
        case drive:
            StartOperatorState();
            OperatorPeriodic();
            if (_oi_driver.GetCoralPickup()) {
                _driver_robot_state = auto_pickup_coral_driver;
                CancelDriverCommands();
                
                _drive_to_feeder_station = _drivetrain.GoToPose(_drivetrain.GetClosestFeederStation());
                _drive_to_feeder_station.Schedule();

            } else if (_oi_driver.GetAlgaePickup()) {
                _driver_robot_state = auto_pickup_algae_driver;
                CancelDriverCommands();

                _drive_to_reef = _drivetrain.GoToPose(_drivetrain.GetClosestReefSide(ReefAlignment::center));
                _drive_to_reef.Schedule();

            } else if (_oi_driver.GetScoreReef()) {
                _driver_robot_state = auto_score_reef_driver;
                CancelDriverCommands();

                _drive_to_reef = _drivetrain.GoToPose(_drivetrain.GetClosestReefSide(_oi_operator.GetReefAlignment()));
                _drive_to_reef.Schedule();
                
            } else if (_oi_driver.GetScoreProcessor()) {
                _driver_robot_state = auto_score_processor_driver;
                

                _drive_to_processor = _drivetrain.GoToPose(_drivetrain.GetClosestProcessor());
                _drive_to_processor.Schedule();
            }
            break;
        case auto_pickup_coral_driver:
            
            if (!_oi_driver.GetCoralPickup()) {CancelDriverCommands(); StartDriveState(); }
            break;
        case auto_pickup_algae_driver:
            
            if (!_oi_driver.GetAlgaePickup() && _oi_operator.GetReefLevel() == 0) { CancelDriverCommands(); StartDriveState(); }
            break;
        case auto_score_reef_driver:
            
            if (!_oi_driver.GetScoreReef() && _oi_operator.GetReefLevel() == 0) { CancelDriverCommands(); StartDriveState(); }
            break;
        case auto_score_processor_driver:
            
            if (!_oi_driver.GetScoreProcessor()) { CancelDriverCommands(); StartDriveState(); }
            break;
        default:
            _driver_robot_state = drive;
    }
}

void Robot::OperatorPeriodic() {
    switch (_operator_drive_robot_state){
        case stow:
            
            //auto 
            if (_oi_operator.GetLoadCoral() && _driver_robot_state == auto_pickup_coral_driver) {
                _intake_coral_commands.Schedule();

                _operator_drive_robot_state = auto_pickup_coral_operator;
            } else if (((_oi_operator.GetReefLevel() == 2 || _oi_operator.GetReefLevel() == 3) && _oi_operator.GetReefAlignment() == ReefAlignment::center) && _driver_robot_state == auto_pickup_algae_driver) {
                _intake_algae_commands.Schedule();

                _operator_drive_robot_state = auto_pickup_algae_operator;
            } else if ((!(_oi_operator.GetReefLevel() == 0) && (_oi_operator.GetReefAlignment() == ReefAlignment::left || _oi_operator.GetReefAlignment() == ReefAlignment::right)) && _driver_robot_state == auto_score_reef_driver) {
                _score_coral_commands.Schedule();

                _operator_drive_robot_state = auto_score_reef_operator;
            } else if (_oi_operator.GetProcessor() && _driver_robot_state == auto_score_processor_driver) {
                _processor_commands.Schedule();

                _operator_drive_robot_state = auto_score_processor_operator;
            }
            
            //manual
            else if (_oi_operator.GetLoadCoral()){
                _intake_coral_commands.Schedule();

                _operator_drive_robot_state = manual_pickup_coral;
            } else if (_oi_operator.GetReefLevel() != 0) {
                _score_coral_commands.Schedule();

                _operator_drive_robot_state = manual_score_coral;
            } else if (_oi_operator.GetProcessor()) {
                _processor_commands.Schedule();

                _operator_drive_robot_state = manual_score_processor;
            } else if ((_oi_operator.GetReefLevel() == 2 || _oi_operator.GetReefLevel() == 3) && _oi_operator.GetReefAlignment() == ReefAlignment::center) {
                _score_coral_commands.Schedule();

                _operator_drive_robot_state = manual_remove_algae;
            } else if (_oi_operator.GetClimbUp()) {
                _climb_up_state_commands.Schedule();

                _operator_drive_robot_state = climb_up;
            } else if (_oi_operator.GetClimbDown()) {
                _climb_down_state_commands.Schedule();

                _operator_drive_robot_state = climb_down;
            }
            break;
        case auto_pickup_coral_operator:
            
            if (!_oi_operator.GetLoadCoral()) {CancelOperatorCommands(); StartOperatorState();}
            break;
        case auto_pickup_algae_operator:

            if (_oi_operator.GetReefLevel() == 0) {CancelOperatorCommands(); StartOperatorState();}
            break;
        case auto_score_reef_operator:

            if (_oi_operator.GetReefLevel() == 0) {CancelOperatorCommands(); StartOperatorState();}
            break;
        case auto_score_processor_operator:

            if (!_oi_operator.GetProcessor()) {CancelOperatorCommands(); StartOperatorState();}
            break;
        case manual_pickup_coral:

            if (!_oi_operator.GetLoadCoral()) {CancelOperatorCommands(); StartOperatorState();}
            break;
        case manual_score_coral:
            
            if (_oi_operator.GetReefLevel() == 0) {CancelOperatorCommands(); StartOperatorState();}
            break;
        case manual_score_processor:

            if (!_oi_operator.GetProcessor()) {CancelOperatorCommands(); StartOperatorState();}
            break;
        case manual_remove_algae:

            if (_oi_operator.GetReefLevel() == 0) {CancelOperatorCommands(); StartOperatorState();}
            break;
        case climb_up:

            if (!_oi_operator.GetClimbUp()) {CancelOperatorCommands(); StartOperatorState();}
            break;
        case climb_down:

            if (!_oi_operator.GetClimbDown()) {CancelOperatorCommands(); StartOperatorState();}
            break;
        default:
            _operator_drive_robot_state = stow;
    }
}

void Robot::StartDriveState() {
    _driver_robot_state = drive;
    _drive_state_commands.Schedule();
}

void Robot::CancelDriverCommands() {
    _drive_state_commands.Cancel();
    _drive_to_feeder_station.Cancel();
    _drive_to_reef.Cancel();
    _drive_to_processor.Cancel();
}

void Robot::CancelOperatorCommands() {
    _stow_state_commands.Cancel();
    _intake_algae_commands.Cancel();
    _intake_coral_commands.Cancel();
    _processor_commands.Cancel();
    _score_coral_commands.Cancel();
}
    
void Robot::StartOperatorState() {
    _operator_drive_robot_state = stow;
    _stow_state_commands.Schedule();
    
    _intake_algae_commands.Cancel();
    _intake_coral_commands.Cancel();
    _processor_commands.Cancel();
    _score_coral_commands.Cancel();
    _climb_down_state_commands.Cancel();
    _climb_up_state_commands.Cancel();
}

void Robot::TeleopExit() {
    CancelDriverCommands();
    CancelOperatorCommands();
    _driver_robot_state = drive;
    _operator_drive_robot_state = stow;
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
