#include "commands/teleop/TeleopIntakeCommand.h"

TeleopIntakeCommand::TeleopIntakeCommand(
    DrivetrainSubsystem* drivetrain, 
    ElevatorSubsystem* elevator, 
    IntakeSubsystem* intake, 
    PivotSubsystem* pivot, 
    FunnelSubsystem* funnel, 
    Operator_Interface* oi) : 
    _drivetrain(drivetrain),
    _elevator(elevator),
    _intake(intake),
    _pivot(pivot),
    _funnel(funnel),
    _oi(oi) {
    AddRequirements(_drivetrain);
    AddRequirements(_elevator);
    AddRequirements(_intake);
    AddRequirements(_pivot);
    AddRequirements(_funnel);
}

// Called when the command is initially scheduled.
void TeleopIntakeCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void TeleopIntakeCommand::Execute() {
    switch(_auto_intake_state) {
        case wait:
            _elevator->SetHeight(ElevatorConstants::HOME_POSITION);
            _pivot->SetPivotAngle(PivotConstants::HOME_POSITION);

            if(_drivetrain->GetNearTargetPosition() || _oi->IgnoreVision()){
                _auto_intake_state = intake;
            }
            break;
        case intake:
            _intake->SetPower(IntakeConstants::EJECT_POWER);
            _funnel->SetPower(FunnelSubsystemConstants::INTAKE_POWER);

            if (_intake->CoralHigh() && _intake->CoralLow()) {
                _auto_intake_state = done;
            }
            break;
        case done:
            _intake->SetPower(IntakeConstants::STOP_POWER);
            _funnel->SetPower(FunnelSubsystemConstants::STOP_POWER);
            break;
        default:
            _auto_intake_state = wait;
            break;
    }
}

// Called once the command ends or is interrupted.
void TeleopIntakeCommand::End(bool interrupted) {
    _intake->SetPower(IntakeConstants::STOP_POWER);
    _funnel->SetPower(FunnelSubsystemConstants::STOP_POWER);
}

// Returns true when the command should end.
bool TeleopIntakeCommand::IsFinished() {
    return _auto_intake_state == done;
}
