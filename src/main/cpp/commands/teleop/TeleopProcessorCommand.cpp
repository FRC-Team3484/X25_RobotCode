#include "commands/teleop/TeleopProcessorCommand.h"
#include "Constants.h"

TeleopProcessorCommand::TeleopProcessorCommand(
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
void TeleopProcessorCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void TeleopProcessorCommand::Execute() {
    switch (_auto_processor_state) {
        case wait:
            if (_drivetrain->GetNearTargetPosition() || _oi->IgnoreVision()) {
                _auto_processor_state = extend_elevator;
            }
            break; 
        case extend_elevator:
            _elevator->SetHeight(ElevatorConstants::PROCESSOR_POSITION);
            if (_elevator->AtTargetHeight() && _drivetrain->GetAtTargetPosition()) {
                _auto_processor_state = extend_pivot;
            }
            break;
        case extend_pivot:
            _pivot->SetPivotAngle(PivotConstants::PROCESSOR_POSITION);
            if (_pivot->AtTargetPosition()) {
                _auto_processor_state = eject_algae;
            }
            break;
        case eject_algae:
            _intake->SetPower(IntakeConstants::EJECT_POWER);
            if (_intake->AlgaeTop()) {
                _auto_processor_state = done;
            }
            break;
        case done:
            _intake->SetPower(IntakeConstants::STOP_POWER);
            break;
        default:
            _auto_processor_state = wait;
            break;
    }
}

// Called once the command ends or is interrupted.
void TeleopProcessorCommand::End(bool interrupted) {
    _intake->SetPower(IntakeConstants::STOP_POWER);
}

// Returns true when the command should end.
bool TeleopProcessorCommand::IsFinished() {
    return _auto_processor_state == done;
}
