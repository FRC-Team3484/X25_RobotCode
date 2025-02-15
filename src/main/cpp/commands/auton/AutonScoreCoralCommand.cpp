#include "commands/auton/AutonScoreCoralCommand.h"
#include "Constants.h"

AutonScoreCoralCommand::AutonScoreCoralCommand(
    DrivetrainSubsystem* drivetrain, 
    ElevatorSubsystem* elevator,
    IntakeSubsystem* intake, 
    PivotSubsystem* pivot) : 
    _drivetrain(drivetrain), 
    _elevator(elevator),
    _intake(intake),
    _pivot(pivot) {
    AddRequirements(_drivetrain);
    AddRequirements(_elevator);
    AddRequirements(_intake);
    AddRequirements(_pivot);
}

// Called when the command is initially scheduled.
void AutonScoreCoralCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void AutonScoreCoralCommand::Execute() {
  switch(_auton_score_coral_state) {
        case wait:
            if(_drivetrain->GetNearTargetPosition()) {
                _auton_score_coral_state = extend_elevator;
            }
            break;
        case extend_elevator:
            _elevator->SetHeight(ElevatorConstants::PROCESSOR_POSITION_3);

            if(_elevator->AtTargetHeight() && _drivetrain->GetAtTargetPosition()) {
                _auton_score_coral_state = extend_pivot;
            }
            break;
        case extend_pivot:
            _pivot->SetPivotAngle(PivotConstants::TARGET_POSITION);

            if(_pivot->AtTargetPosition()) {
                _auton_score_coral_state = eject_piece;
            }
            break;
        case eject_piece:
            _intake->SetPower(IntakeConstants::EJECT_POWER);

            if(!_intake->CoralHigh() && !_intake->CoralLow()) {
                _auton_score_coral_state = done;
            }
            break;
        case done:
            _intake->SetPower(IntakeConstants::STOP_POWER);
            break;
        default:
            _auton_score_coral_state = wait;
            break;
    }
}

// Called once the command ends or is interrupted.
void AutonScoreCoralCommand::End(bool interrupted) {
    _intake->SetPower(IntakeConstants::STOP_POWER);
}

// Returns true when the command should end.
bool AutonScoreCoralCommand::IsFinished() {
    return _auton_score_coral_state == done;
}
