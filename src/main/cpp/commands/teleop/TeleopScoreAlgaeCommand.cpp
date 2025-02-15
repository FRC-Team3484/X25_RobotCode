#include "commands/teleop/TeleopScoreAlgaeCommand.h"
#include "Constants.h"

TeleopScoreAlgaeCommand::TeleopScoreAlgaeCommand(
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
void TeleopScoreAlgaeCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void TeleopScoreAlgaeCommand::Execute() {
	switch(_auto_score_algae_state) {
		case wait:
			if(_drivetrain->GetNearTargetPosition() || _oi->IgnoreVision()) {
                _auto_score_algae_state = extend_elevator;
            }
			break;
		case extend_elevator:
            _elevator->SetHeight(ElevatorConstants::PROCESSOR_POSITION_2);

            if(_elevator->AtTargetHeight() && _drivetrain->GetAtTargetPosition()) {
                _auto_score_algae_state = extend_pivot;
            }
            break;
        case extend_pivot:
            _pivot->SetPivotAngle(PivotConstants::TARGET_POSITION);

            if(_pivot->AtTargetPosition()) {
                _auto_score_algae_state = eject_piece;
            }
            break;
        case eject_piece:
            _intake->SetPower(IntakeConstants::EJECT_POWER);

            if(!_intake->AlgaeTop() && !_intake->AlgaeBottom()) {
                _auto_score_algae_state = done;
            }
            break;
        case done:
            _intake->SetPower(IntakeConstants::STOP_POWER);
            break;
        default:
            _auto_score_algae_state = wait;
            break;
	}
}

// Called once the command ends or is interrupted.
void TeleopScoreAlgaeCommand::End(bool interrupted) {
	_intake->SetPower(IntakeConstants::STOP_POWER);
}

// Returns true when the command should end.
bool TeleopScoreAlgaeCommand::IsFinished() {
	return _auto_score_algae_state == done;
}
