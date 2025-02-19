#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <frc/Filesystem.h>
#include <wpinet/WebServer.h>

Robot::Robot() {
    wpi::WebServer::GetInstance().Start(5800, frc::filesystem::GetDeployDirectory());
}

void Robot::RobotPeriodic() {
    frc2::CommandScheduler::GetInstance().Run();

    frc::SmartDashboard::PutNumber("Voltage", frc::DriverStation::GetBatteryVoltage());
    
    _match_time = frc::DriverStation::GetMatchTime();
    frc::SmartDashboard::PutNumber("Match Time", _match_time.to<double>());
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
    _drive_state_commands.Schedule();
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

void Robot::TestInit() {
    frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif
