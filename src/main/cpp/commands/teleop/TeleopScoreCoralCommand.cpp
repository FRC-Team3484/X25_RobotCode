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

// Called when the command is initially scheduled.
void TeleopScoreCoralCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void TeleopScoreCoralCommand::Execute() {
    switch(_auto_score_coral_state) {
        case wait:
            if(_drivetrain->GetNearTargetPosition() || _oi->IgnoreVision()) {
                _auto_score_coral_state = extend_elevator;
            }
            break;
        case extend_elevator:
            if (_oi->GetReefLevel() == 1) {
                _elevator->SetHeight(ElevatorConstants::CORAL_LEVEL_1);
            } else if (_oi->GetReefLevel() == 2) {
                _elevator->SetHeight(ElevatorConstants::CORAL_LEVEL_2);
            } else if (_oi->GetReefLevel() == 3) {
                _elevator->SetHeight(ElevatorConstants::CORAL_LEVEL_3);
            } else if (_oi->GetReefLevel() == 4) {
                _elevator->SetHeight(ElevatorConstants::CORAL_LEVEL_4);
            }

            if(_elevator->AtTargetHeight() && _drivetrain->GetAtTargetPosition()) {
                _auto_score_coral_state = extend_pivot;
            }
            break;
        case extend_pivot:
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
            _intake->SetPower(IntakeConstants::EJECT_POWER);

            if(!_intake->HasCoral()) {
                _auto_score_coral_state = done;
            }
            break;
        case done:
            _intake->SetPower(IntakeConstants::STOP_POWER);
            break;
        default:
            _auto_score_coral_state = wait;
            break;
    }
}

// Called once the command ends or is interrupted.
void TeleopScoreCoralCommand::End(bool interrupted) {
    _intake->SetPower(IntakeConstants::STOP_POWER);
}

// Returns true when the command should end.
bool TeleopScoreCoralCommand::IsFinished() {
    return _auto_score_coral_state == done;
}
