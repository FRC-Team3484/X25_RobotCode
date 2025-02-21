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
            _stow_state_commands.Cancel();
            OperatorPeriodic();
            _drive_state_commands.Schedule();
            break;
        case auto_pickup_coral:
            _drive_state_commands.Cancel();
            _test_state_commands.Cancel();
            
            _stow_state_commands.Schedule();
            break;
        case auto_score_reef:
            
            break;
        case auto_score_processor:

            break;
        default:
            _driver_robot_state = drive;
    }
}

void Robot::OperatorPeriodic() {
    switch (_operator_drive_robot_state){
        case stow:
        
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

void Robot::TeleopExit() {}

void Robot::TestInit() {
    frc2::CommandScheduler::GetInstance().CancelAll();
    _test_state_commands.Schedule();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif
