#include "commands/teleop/ClimbUpCommand.h"

ClimbUpCommand::ClimbUpCommand(
	ElevatorSubsystem* elevator,
    PivotSubsystem* pivot) : 
	_elevator(elevator),
    _pivot(pivot) {
    AddRequirements(_elevator);
    AddRequirements(_pivot);
}

void ClimbUpCommand::Initialize() {
    _climb_up_state = traveling_pivot;
}

void ClimbUpCommand::Execute() {
    switch (_climb_up_state) {
        case traveling_pivot:
            // Stow the pivot
            // When the pivot has been stowed, set the state to climb_up_elevator
            _pivot->SetPivotAngle(PivotConstants::TRAVEL_POSITION);
            if (_pivot->AtTargetPosition()) {
                _climb_up_state = climb_up_elevator;
            }
            break;
        
        case climb_up_elevator:
            // Stow the elevator
            // When the elevator has been stowed, set the state to done
            _elevator->SetHeight(ElevatorConstants::CLIMB_HEIGHT);
            if (_elevator->AtTargetHeight()) {
                _climb_up_state = done;
            }
            break;

        case done:
            // End the command
            break;
        
        default:
            _climb_up_state = traveling_pivot;
    }
}

void ClimbUpCommand::End(bool interrupted) {}

bool ClimbUpCommand::IsFinished() {
    return _climb_up_state == done;
}
