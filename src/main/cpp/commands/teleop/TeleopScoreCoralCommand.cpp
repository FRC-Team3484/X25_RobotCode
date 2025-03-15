#include "commands/teleop/TeleopScoreCoralCommand.h"
#include "Constants.h"

TeleopScoreCoralCommand::TeleopScoreCoralCommand(
    DrivetrainSubsystem* drivetrain, 
    ElevatorSubsystem* elevator,
    IntakeSubsystem* intake, 
    PivotSubsystem* pivot,   
    Operator_Interface* oi) : 
    _drivetrain(drivetrain), 
    _elevator(elevator),
    _intake(intake),
    _pivot(pivot), 
    _oi(oi) {
    AddRequirements(_drivetrain);
    AddRequirements(_elevator);
    AddRequirements(_intake);
    AddRequirements(_pivot);
}

void TeleopScoreCoralCommand::Initialize() {
    _auto_score_coral_state = wait;
}

void TeleopScoreCoralCommand::Execute() {
    switch(_auto_score_coral_state) {
        case wait:
            // Wait until the robot is near the target position, then go to next state
            if(_drivetrain->GetNearTargetPosition() || _oi->IgnoreVision()) {
                _auto_score_coral_state = traveling_pivot;
            }
            break;
        case traveling_pivot:
            // Set the pivot to the travel position
            // Once the pivot is at it's target position, go to the next state
            _pivot->SetPivotAngle(PivotConstants::TRAVEL_POSITION);
            if (_pivot->AtTargetPosition()) {
                _auto_score_coral_state = extend_elevator;
            }
            break;
        case extend_elevator:
            // Set the height of the elevator based off of the selected reef level
            // Once the elevator is at the target height and the drivetrain has reached the target position, go to the next state
            if (_oi->GetReefLevel() == 1) {
                _elevator->SetHeight(ElevatorConstants::CORAL_LEVEL_1);
            } else if (_oi->GetReefLevel() == 2) {
                _elevator->SetHeight(ElevatorConstants::CORAL_LEVEL_2);
            } else if (_oi->GetReefLevel() == 3) {
                _elevator->SetHeight(ElevatorConstants::CORAL_LEVEL_3);
            } else if (_oi->GetReefLevel() == 4) {
                _elevator->SetHeight(ElevatorConstants::CORAL_LEVEL_4);
            }

            if(_elevator->AtTargetHeight() && (_drivetrain->GetAtTargetPosition() || _oi->IgnoreVision())) {
                _auto_score_coral_state = extend_pivot;
            }
            break;
        case extend_pivot:
            // Set the angle of the pivot to the coral angle depending on the selected reef level
            // Once the pivot is at the target angle, go to the next state
            if (_oi->GetReefLevel() == 1 || _oi->GetReefLevel() == 2 ||_oi->GetReefLevel() == 3){
                _pivot->SetPivotAngle(PivotConstants::TARGET_CORAL_ANGLE);
            } else if (_oi->GetReefLevel() == 4) {
                _pivot->SetPivotAngle(PivotConstants::TARGET_CORAL_4_ANGLE);
            }

            if(_pivot->AtTargetPosition()) {
                _auto_score_coral_state = eject_piece;
            }
            break;
        case eject_piece:
            // Run the intake to eject the coral
            // Once the intake no longer has coral, go to the next state
            _intake->SetPower(IntakeConstants::CORAL_EJECT_POWER);

            if(!_intake->HasCoral()) {
                _auto_score_coral_state = done;
            }
            break;
        case done:
            // Stop the intake
            _intake->SetPower(IntakeConstants::STOP_POWER);
            break;
        default:
            _auto_score_coral_state = wait;
            break;
    }
}

void TeleopScoreCoralCommand::End(bool interrupted) {
    _intake->SetPower(IntakeConstants::STOP_POWER);
}

bool TeleopScoreCoralCommand::IsFinished() {
    return _auto_score_coral_state == done;
}
