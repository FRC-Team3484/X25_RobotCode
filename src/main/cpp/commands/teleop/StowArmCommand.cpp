#include "commands/teleop/StowArmCommand.h"

#include <frc/smartdashboard/SmartDashboard.h>

StowArmCommand::StowArmCommand(
	PivotSubsystem* pivot, 
	ElevatorSubsystem* elevator) : 
	_pivot(pivot), 
	_elevator(elevator) {
    AddRequirements(_pivot);
    AddRequirements(_elevator);
}

void StowArmCommand::Initialize() {
    _elevator->PrintTestInfo();
    _pivot->PrintTestInfo();
    if (_pivot->PivotDeployed())_stow_arm_state = traveling_pivot;
    else _stow_arm_state = stow_elevator;
}

void StowArmCommand::Execute() {
    _elevator->PrintTestInfo();
    
    _pivot->PrintTestInfo();
    switch (_stow_arm_state) {
        case traveling_pivot:
            // Stow the arm
            // When the arm has been stowed, start stowing the elevator
            _pivot->SetPivotAngle(PivotConstants::TRAVEL_POSITION);
            // fmt::println("Traveling Pivot Stow");
            if (_pivot->AtTargetPosition()) {
                _stow_arm_state = stow_elevator;
            }
            break;

        case stow_elevator:
            // Stow the elevator
            // When the elevator has been stowed, set the state to done
            _elevator->SetHeight(ElevatorConstants::HOME_POSITION);
            _elevator->PrintTestInfo();
            _pivot->PrintTestInfo();
            //fmt::println("Stow Elevator");
            _elevator->PrintTestInfo();
            if (_elevator->AtSafeStowPosition()) {
                _stow_arm_state = stow_arm;
            }
            break;
        case stow_arm:
            // Stow the pivot
            // When the pivot has been stowed, set the state to done
            _pivot->SetPivotAngle(PivotConstants::HOME_POSITION);
            _elevator->PrintTestInfo();
            _pivot->PrintTestInfo();
            //fmt::println("Stow Pivot");
            if (_pivot->AtTargetPosition()) {
                _stow_arm_state = done;
            }
            break;
        case done:
            // End the command
            break;
        
        default:
            _stow_arm_state = stow_elevator;
    }
}

void StowArmCommand::End(bool interrupted) {}

bool StowArmCommand::IsFinished() {
    //if(_stow_arm_state == done) fmt::println("Done");
    return _stow_arm_state == done;
}
