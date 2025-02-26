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
    OperatorPeriodic();
    switch (_driver_robot_state) {
        case drive:
            if (_oi_driver.GetCoralPickup()) {
                _driver_robot_state = auto_pickup_coral_driver;
                CancelDriverCommands();
                
                _drive_to_feeder_station = _drivetrain.GoToPose(_drivetrain.GetClosestFeederStation());
                _drive_to_feeder_station.Schedule();

            } else if ((_oi_driver.GetScoreReef() || _oi_driver.GetAlgaePickup()) && !_oi_operator.GetReefLevel() == 0) {
                _driver_robot_state = auto_reef_driver;
                CancelDriverCommands();

                _drive_to_reef = _drivetrain.GoToPose(_drivetrain.GetClosestReefSide(_oi_operator.GetReefAlignment()));
                _drive_to_reef.Schedule();

            } else if (_oi_driver.GetScoreProcessor()) {
                _driver_robot_state = auto_score_processor_driver;
                

                _drive_to_processor = _drivetrain.GoToPose(_drivetrain.GetClosestProcessor());
                _drive_to_processor.Schedule();
            }
            break;

        //auto
        case auto_pickup_coral_driver:
            
            if (!_oi_driver.GetCoralPickup() && !_oi_operator.GetLoadCoral()) {CancelDriverCommands(); StartDriveState(); }
            break;
        case auto_reef_driver:
            
            if (!(_oi_driver.GetScoreReef() || _oi_driver.GetAlgaePickup()) && _oi_operator.GetReefLevel() == 0) { CancelDriverCommands(); StartDriveState(); }
            break;
        case auto_score_processor_driver:
            
            if (!_oi_driver.GetScoreProcessor() && !_oi_operator.GetProcessor()) { CancelDriverCommands(); StartDriveState(); }
            break;
        default:
            _driver_robot_state = drive;
    }
}

void Robot::OperatorPeriodic() {
    switch (_operator_drive_robot_state){
        case stow:
            
            //auto 
            if (AutoGetLoadCoralCondition()) {
                _intake_coral_commands.Schedule();

                _operator_drive_robot_state = auto_pickup_coral_operator;
            } else if (AutoGetRemoveAlgaeCondition()) {
                _intake_algae_commands.Schedule();

                _operator_drive_robot_state = auto_pickup_algae_operator;
            } else if (AutoGetScoreReefCondition()) {
                _score_coral_commands.Schedule();

                _operator_drive_robot_state = auto_score_reef_operator;
            } else if (AutoGetScoreProcessorCondition()) {
                _processor_commands.Schedule();

                _operator_drive_robot_state = auto_score_processor_operator;
            }
            
            //manual
            else if (ManualGetLoadCoralCondition()) {
                _intake_coral_commands.Schedule();

                _operator_drive_robot_state = manual_pickup_coral;
            } else if (ManualGetScoreReefCondition()) {
                _score_coral_commands.Schedule();

                _operator_drive_robot_state = manual_score_coral;
            } else if (ManualGetScoreProcessorCondition()) {
                _processor_commands.Schedule();

                _operator_drive_robot_state = manual_score_processor;
            } else if (ManualGetRemoveAlgaeCondition()) {
                _score_coral_commands.Schedule();

                _operator_drive_robot_state = manual_remove_algae;
            } else if (ManualGetClimbUpCondition()) {
                _climb_up_state_commands.Schedule();

                _operator_drive_robot_state = climb_up;
            }
            break;

        //auto
        case auto_pickup_coral_operator:
            
            if (!AutoGetLoadCoralCondition()) {CancelOperatorCommands(); StartOperatorState();}
            break;
        case auto_pickup_algae_operator:

            if (!AutoGetRemoveAlgaeCondition()) {CancelOperatorCommands(); StartOperatorState();}
            break;
        case auto_score_reef_operator:

            if (!AutoGetScoreReefCondition()) {CancelOperatorCommands(); StartOperatorState();}
            break;
        case auto_score_processor_operator:

            if (!AutoGetScoreProcessorCondition()) {CancelOperatorCommands(); StartOperatorState();}
            break;

        //manual
        case manual_pickup_coral:

            if (!ManualGetLoadCoralCondition()) {CancelOperatorCommands(); StartOperatorState();}
            break;
        case manual_score_coral:
            
            if (!ManualGetScoreReefCondition()) {CancelOperatorCommands(); StartOperatorState();}
            break;
        case manual_score_processor:

            if (!ManualGetScoreProcessorCondition()) {CancelOperatorCommands(); StartOperatorState();}
            break;
        case manual_remove_algae:

            if (!ManualGetRemoveAlgaeCondition()) {CancelOperatorCommands(); StartOperatorState();}
            break;
        case climb_up:

            if (!ManualGetClimbUpCondition()) {CancelOperatorCommands(); StartOperatorState();}
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
    _climb_up_state_commands.Cancel();
}
    
void Robot::StartOperatorState() {
    _operator_drive_robot_state = stow;
    _stow_state_commands.Schedule();
}

bool Robot::AutoGetLoadCoralCondition(){
    return _oi_operator.GetLoadCoral() && _driver_robot_state == auto_pickup_coral_driver && (!_intake.HasCoral());
}
bool Robot::AutoGetRemoveAlgaeCondition(){
    return ((_oi_operator.GetReefLevel() == 2 || _oi_operator.GetReefLevel() == 3) && _oi_operator.GetReefAlignment() == ReefAlignment::center) && _driver_robot_state == auto_reef_driver && (!_intake.HasAlgae());
}
bool Robot::AutoGetScoreReefCondition(){
    return ((!(_oi_operator.GetReefLevel() == 0) && (_oi_operator.GetReefAlignment() == ReefAlignment::left || _oi_operator.GetReefAlignment() == ReefAlignment::right)) && _driver_robot_state == auto_reef_driver) && (_intake.HasCoral());
}
bool Robot::AutoGetScoreProcessorCondition(){
    return _oi_operator.GetProcessor() && _driver_robot_state == auto_score_processor_driver && (_intake.HasAlgae());
}

bool Robot::ManualGetLoadCoralCondition(){
    return _oi_operator.GetLoadCoral() && _driver_robot_state == drive && (!_intake.HasCoral());
}
bool Robot::ManualGetRemoveAlgaeCondition(){
    return ((_oi_operator.GetReefLevel() == 2 || _oi_operator.GetReefLevel() == 3) && _oi_operator.GetReefAlignment() == ReefAlignment::center) && _driver_robot_state == drive && (!_intake.HasAlgae());
}
bool Robot::ManualGetScoreReefCondition(){
    return ((!(_oi_operator.GetReefLevel() == 0) && (_oi_operator.GetReefAlignment() == ReefAlignment::left || _oi_operator.GetReefAlignment() == ReefAlignment::right)) && _driver_robot_state == drive) && (_intake.HasCoral());
}
bool Robot::ManualGetScoreProcessorCondition(){
    return _oi_operator.GetProcessor() && _driver_robot_state == drive && (_intake.HasAlgae());
}
bool Robot::ManualGetClimbUpCondition(){
    return _oi_operator.GetClimbUp() && _driver_robot_state == drive;
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
