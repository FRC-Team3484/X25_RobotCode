#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>

Robot::Robot() {}

void Robot::RobotPeriodic() {
    frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() {
    _leds.StartIdleAnimation();

    frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::DisabledPeriodic() {
    if (_pdp.GetVoltage() < 12.2) {
        _low_battery = true;
    }
    if (_low_battery){
        _leds->FireAnimation();
    } else if (_has_been_enabled) {
        _leds->WaveAnimation();
    } else {
        _leds->SandAnimation();
    }
}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
    _has_been_enabled =true;
}

void Robot::AutonomousPeriodic() {
    _leds->TetrisAnimation();
}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
    StartDriveState();
    _has_been_enabled = true;
    StartOperatorState();
}

void Robot::TeleopPeriodic() {
    OperatorPeriodic();
    if (_intake->HasCoral())_leds->HasCoralAnimation(); else if (_intake->HasAlgae()) _leds->HasAlgaeAnimation();
    switch (_driver_robot_state) {
        case drive:
            _leds->DrivingAnimation();
            if (_oi_driver->GetDynamicPivot()){
                _leds->PivotAnimation();
            } else if (_oi_driver->GetCoralPickup()) {
                _driver_robot_state = auto_pickup_coral_driver;
                CancelDriverCommands();

                _drive_to_feeder_station = _drivetrain->GoToPose(_drivetrain->GetClosestFeederStation());
                _drive_to_feeder_station.Schedule();

            } else if ((_oi_driver->GetScoreReef() || _oi_driver->GetAlgaePickup()) && _oi_operator->GetReefLevel() != 0) {
                _driver_robot_state = auto_reef_driver;
                CancelDriverCommands();

                _drive_to_reef = _drivetrain->GoToPose(_drivetrain->GetClosestReefSide(_oi_operator->GetReefAlignment()));
                _drive_to_reef.Schedule();

            } else if (_oi_driver->GetScoreProcessor()) {
                _driver_robot_state = auto_score_processor_driver;
                CancelDriverCommands();

                _drive_to_processor = _drivetrain->GoToPose(_drivetrain->GetClosestProcessor());
                _drive_to_processor.Schedule();
            }
            break;

        //auto
        case auto_pickup_coral_driver:
            _leds->PathAnimation();
            
            if (!_oi_driver->GetCoralPickup()) {CancelDriverCommands(); StartDriveState(); }
            break;

        case auto_reef_driver:
            _leds->PathAnimation();
            
            if (!(_oi_driver->GetScoreReef() || _oi_driver->GetAlgaePickup())) { CancelDriverCommands(); StartDriveState(); }
            break;
        case auto_score_processor_driver:
            _leds->PathAnimation();
            
            if (!_oi_driver->GetScoreProcessor()) { CancelDriverCommands(); StartDriveState(); }
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
                CancelOperatorCommands();
                _intake_coral_commands.Schedule();

                _operator_drive_robot_state = auto_pickup_coral_operator;
            } else if (AutoGetRemoveAlgaeCondition()) {
                CancelOperatorCommands();
                _intake_algae_commands.Schedule();

                _operator_drive_robot_state = auto_pickup_algae_operator;
            } else if (AutoGetScoreReefCondition()) {
                CancelOperatorCommands();
                _score_coral_commands.Schedule();

                _operator_drive_robot_state = auto_score_reef_operator;
            } else if (AutoGetScoreProcessorCondition()) {
                CancelOperatorCommands();
                _processor_commands.Schedule();

                _operator_drive_robot_state = auto_score_processor_operator;
            }
            
            //manual
            else if (ManualGetLoadCoralCondition()) {
                CancelOperatorCommands();
                _intake_coral_commands.Schedule();

                _operator_drive_robot_state = manual_pickup_coral;
            } else if (ManualGetScoreReefCondition()) {
                CancelOperatorCommands();
                _score_coral_commands.Schedule();

                _operator_drive_robot_state = manual_score_coral;
            } else if (ManualGetScoreProcessorCondition()) {
                CancelOperatorCommands();
                _processor_commands.Schedule();

                _operator_drive_robot_state = manual_score_processor;
            } else if (ManualGetRemoveAlgaeCondition()) {
                CancelOperatorCommands();
                _score_coral_commands.Schedule();

                _operator_drive_robot_state = manual_remove_algae;
            } else if (ManualGetClimbUpCondition()) {
                CancelOperatorCommands();
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
            _leds->ScoringAnimation();

            if (!AutoGetScoreReefCondition()) {CancelOperatorCommands(); StartOperatorState();}
            break;

        case auto_score_processor_operator:
            _leds->ScoringAnimation();

            if (!AutoGetScoreProcessorCondition()) {CancelOperatorCommands(); StartOperatorState();}
            break;

        //manual
        case manual_pickup_coral:
            if (!ManualGetLoadCoralCondition()) {CancelOperatorCommands(); StartOperatorState();}
            break;
        case manual_score_coral:
            _leds->ScoringAnimation();
            
            if (!ManualGetScoreReefCondition()) {CancelOperatorCommands(); StartOperatorState();}
            break;

        case manual_score_processor:
            _leds->ScoringAnimation();

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

// This functiion is called to enter the drive state in teleop (the default state)
void Robot::StartDriveState() {
    _driver_robot_state = drive;
    _drive_state_commands.Schedule();
}

// This functiion is called to enter the stow state in teleop (the default state)
void Robot::StartOperatorState() {
    _operator_drive_robot_state = stow;
    _stow_state_commands.Schedule();
}

void Robot::CancelDriverCommands() {
    if (_drive_state_commands.IsScheduled()) _drive_state_commands.Cancel();
    if (_drive_to_feeder_station.IsScheduled()) _drive_to_feeder_station.Cancel();
    if (_drive_to_reef.IsScheduled()) _drive_to_reef.Cancel();
    if (_drive_to_processor.IsScheduled()) _drive_to_processor.Cancel();
}

void Robot::CancelOperatorCommands() {
    _stow_state_commands.Cancel();
    _intake_algae_commands.Cancel();
    _intake_coral_commands.Cancel();
    _processor_commands.Cancel();
    _score_coral_commands.Cancel();
    _climb_up_state_commands.Cancel();
}

// Conditions for the Auto and Manual Commands in the Operator State
bool Robot::AutoGetLoadCoralCondition(){return _oi_operator->GetLoadCoral() && _driver_robot_state == auto_pickup_coral_driver && (!_intake->HasCoral());}
bool Robot::AutoGetRemoveAlgaeCondition(){return ((_oi_operator->GetReefLevel() == 2 || _oi_operator->GetReefLevel() == 3) && _oi_operator->GetReefAlignment() == ReefAlignment::center) && _driver_robot_state == auto_reef_driver && (!_intake->HasAlgae());}
bool Robot::AutoGetScoreReefCondition(){return ((!(_oi_operator->GetReefLevel() == 0) && (_oi_operator->GetReefAlignment() == ReefAlignment::left || _oi_operator->GetReefAlignment() == ReefAlignment::right)) && _driver_robot_state == auto_reef_driver) && (_intake->HasCoral());}
bool Robot::AutoGetScoreProcessorCondition(){return _oi_operator->GetProcessor() && _driver_robot_state == auto_score_processor_driver && (_intake->HasAlgae());}

bool Robot::ManualGetLoadCoralCondition(){return _oi_operator->GetLoadCoral() && _driver_robot_state == drive && (!_intake->CoralLow());}
bool Robot::ManualGetRemoveAlgaeCondition(){return ((_oi_operator->GetReefLevel() == 2 || _oi_operator->GetReefLevel() == 3) && _oi_operator->GetReefAlignment() == ReefAlignment::center) && _driver_robot_state == drive && (!_intake->HasAlgae());}
bool Robot::ManualGetScoreReefCondition(){return ((!(_oi_operator->GetReefLevel() == 0) && (_oi_operator->GetReefAlignment() == ReefAlignment::left || _oi_operator->GetReefAlignment() == ReefAlignment::right)) && _driver_robot_state == drive) && (_intake->HasCoral());}
bool Robot::ManualGetScoreProcessorCondition(){return _oi_operator->GetProcessor() && _driver_robot_state == drive && (_intake->HasAlgae());}
bool Robot::ManualGetClimbUpCondition(){return _oi_operator->GetClimbUp() && _driver_robot_state == drive;}

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
