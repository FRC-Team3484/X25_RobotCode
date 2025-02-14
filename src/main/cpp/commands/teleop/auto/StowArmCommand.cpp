#include "commands/teleop/auto/StowArmCommand.h"

StowArmCommand::StowArmCommand(
	PivotSubsystem* pivot, 
	ElevatorSubsystem* elevator)
    : 
	_pivot(pivot), 
	_elevator(elevator) 
	{
  	AddRequirements(_pivot);
  	AddRequirements(_elevator);
}

void StowArmCommand::Initialize() {
  _pivot->SetPivotAngle(PivotConstants::HOME_POSITION);
  _elevator->SetHeight(ElevatorConstants::HOME_POSITION);
}

void StowArmCommand::Execute() {}

void StowArmCommand::End(bool interrupted) {}

bool StowArmCommand::IsFinished() {
  return false;
}
