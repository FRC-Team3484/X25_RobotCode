#include "commands/auton/AutonBasicScoreCoralCommand.h"
#include "Constants.h"

AutonBasicScoreCoralCommand::AutonBasicScoreCoralCommand(
    ElevatorSubsystem* elevator,
    IntakeSubsystem* intake, 
    PivotSubsystem* pivot) : 
    _elevator(elevator),
    _intake(intake),
    _pivot(pivot) {
    AddRequirements(_elevator);
    AddRequirements(_intake);
    AddRequirements(_pivot);
}

void AutonBasicScoreCoralCommand::Initialize() {}

void AutonBasicScoreCoralCommand::Execute() {
    switch(_auton_score_coral_state) {
        case traveling_pivot:
            // Set the pivot to the travel position
            // Once the pivot is at it's target position, go to the next state
            _pivot->SetPivotAngle(PivotConstants::TRAVEL_POSITION);
            if (_pivot->AtTargetPosition()) {
                _auton_score_coral_state = extend_elevator;
            }
            break;
        case extend_elevator:
            // Set the height of the elevator
            // Once the elevator is at the target height, go to the next state

            _elevator->SetHeight(ElevatorConstants::CORAL_LEVEL_4);
            if (_elevator->AtTargetHeight()) {
                _auton_score_coral_state = extend_pivot;
            }
            break;
        case extend_pivot:
            // Set the angle of the pivot
            // Once the pivot is at the target angle, go to the next state

            _pivot->SetPivotAngle(PivotConstants::TARGET_CORAL_4_ANGLE); 
            if (_pivot->AtTargetPosition()) {
                _auton_score_coral_state = eject_piece;
            }
            break;
        case eject_piece:
            // Run the intake to eject the piece
            // Once the intake no longer has coral, go to the next state
            _intake->SetPower(IntakeConstants::CORAL_EJECT_POWER);
            if (!_intake->HasCoral()) {
                _auton_score_coral_state = done;
            }
            break;
        case done:
            // Stop the intake
            _intake->SetPower(IntakeConstants::STOP_POWER);
            break;
        default:
            _auton_score_coral_state = traveling_pivot;
            break;
    }
}

void AutonBasicScoreCoralCommand::End(bool interrupted) {
    _intake->SetPower(IntakeConstants::STOP_POWER);
}

bool AutonBasicScoreCoralCommand::IsFinished() {
    return _auton_score_coral_state == done;
}
