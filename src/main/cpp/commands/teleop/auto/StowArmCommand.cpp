#include "commands/teleop/auto/StowArmCommand.h"

StowArmCommand::StowArmCommand(
	PivotSubsystem* pivot, 
	ElevatorSubsystem* elevator) : 
	_pivot(pivot), 
	_elevator(elevator) {
    AddRequirements(_pivot);
    AddRequirements(_elevator);
}

void StowArmCommand::Initialize() {
    _pivot->SetPivotAngle(PivotConstants::HOME_POSITION);
}

void StowArmCommand::Execute() {
    switch (_stow_arm_state) {
        case stow_arm:
            // Stow the arm
            // When the arm has been stowed, start stowing the elevator
            _pivot->SetPivotAngle(PivotConstants::HOME_POSITION);
            if (_pivot->AtTargetPosition()) {
                _stow_arm_state = stow_elevator;
            }
            break;

        case stow_elevator:
            // Stow the elevator
            // When the elevator has been stowed, set the state to done
            _elevator->SetHeight(ElevatorConstants::HOME_POSITION);
            if (_elevator->AtTargetHeight()) {
                _stow_arm_state = done;
            }
            break;

        case done:
            // End the command
            break;
        
        default:
            _stow_arm_state = stow_arm;
    }
}

void StowArmCommand::End(bool interrupted) {}

bool StowArmCommand::IsFinished() {
    return _stow_arm_state == done;
}
