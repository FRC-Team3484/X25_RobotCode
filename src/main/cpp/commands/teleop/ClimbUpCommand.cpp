#include "commands/teleop/ClimbUpCommand.h"

ClimbUpCommand::ClimbUpCommand(
	ElevatorSubsystem* elevator) : 
	_elevator(elevator) {
    AddRequirements(_elevator);
}

void ClimbUpCommand::Initialize() {
}

void ClimbUpCommand::Execute() {
    switch (_climb_up_state) {
        case climb_up_elevator:
            // Stow the elevator
            // When the elevator has been stowed, set the state to done
            _elevator->SetHeight(ElevatorConstants::HOME_POSITION);
            if (_elevator->AtTargetHeight()) {
                _climb_up_state = done;
            }
            break;

        case done:
            // End the command
            break;
        
        default:
            _climb_up_state = climb_up_elevator;
    }
}

void ClimbUpCommand::End(bool interrupted) {}

bool ClimbUpCommand::IsFinished() {
    return _climb_up_state == done;
}
