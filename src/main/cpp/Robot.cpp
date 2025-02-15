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
    _robot_state = stow;
}

void Robot::TeleopPeriodic() {
    switch (_robot_state) {
        case drive:
            _test_state_commands.Cancel();
            _stow_state_commands.Cancel();

            _drive_state_commands.Schedule();
            break;
        case stow:
            _drive_state_commands.Cancel();
            _test_state_commands.Cancel();
            
            _stow_state_commands.Schedule();
            break;
        case intake_algae:
            
            break;
        case score_algae:
            
            break;
        case intake_coral:
            
            break;
        case score_coral:

            break;
        default:
            _robot_state = stow;
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
