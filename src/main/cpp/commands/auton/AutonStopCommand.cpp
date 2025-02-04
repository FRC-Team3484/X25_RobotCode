#include "Constants.h"
#include "commands/auton/AutonStopCommand.h"

/* TODO: Inline this command in AutonGenerator */

using namespace frc;

AutonStopCommand::AutonStopCommand(DrivetrainSubsystem* drivetrain_subsystem)
    : _drivetrain_subsystem{drivetrain_subsystem} {
        AddRequirements(_drivetrain_subsystem);
}

void AutonStopCommand::Initialize() {
    _drivetrain_subsystem->StopMotors();
}

void AutonStopCommand::Execute() {}

void AutonStopCommand::End(bool inturrupted) {}

bool AutonStopCommand::IsFinished() {
    return true;
}