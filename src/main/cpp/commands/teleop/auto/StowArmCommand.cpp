#include "commands/teleop/auto/StowArmCommand.h"

StowArmCommand::StowArmCommand(PivotSubsystem* pivot_subsystem, ElevatorSubsystem* elevator_subsystem)
    : _pivot_subsystem(pivot_subsystem), _elevator_subsystem(elevator_subsystem) {}

void StowArmCommand::Initialize() {
  _pivot_subsystem->SetPivotAngle(PivotConstants::HOME_POSITION);
  _elevator_subsystem->SetHeight(ElevatorConstants::HOME_POSITION);
}

void StowArmCommand::Execute() {}

void StowArmCommand::End(bool interrupted) {}

bool StowArmCommand::IsFinished() {
  return false;
}
