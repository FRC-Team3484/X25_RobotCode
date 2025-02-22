#include "commands/teleop/ClimbDownCommand.h"

ClimbDownCommand::ClimbDownCommand(
	ElevatorSubsystem* elevator) : 
	_elevator(elevator) {
    AddRequirements(_elevator);
}

void ClimbDownCommand::Initialize() {
}

void ClimbDownCommand::Execute() {
    switch (_climb_down_state) {
        case climb_down_elevator:
            // Stow the elevator
            // When the elevator has been stowed, set the state to done
            _elevator->SetHeight(ElevatorConstants::HOME_POSITION);
            if (_elevator->AtTargetHeight()) {
                _climb_down_state = done;
            }
            break;

        case done:
            // End the command
            break;
        
        default:
            _climb_down_state = climb_down_elevator;
    }
}

void ClimbDownCommand::End(bool interrupted) {}

bool ClimbDownCommand::IsFinished() {
    return _climb_down_state == done;
}
