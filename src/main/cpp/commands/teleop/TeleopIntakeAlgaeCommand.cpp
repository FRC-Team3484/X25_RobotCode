#include "commands/teleop/TeleopIntakeAlgaeCommand.h"

TeleopIntakeAlgaeCommand::TeleopIntakeAlgaeCommand(
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
void TeleopIntakeAlgaeCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void TeleopIntakeAlgaeCommand::Execute() {
    switch(_auto_intake_algae_state) {
        case wait:
            _elevator->SetHeight(ElevatorConstants::HOME_POSITION);
            _pivot->SetPivotAngle(PivotConstants::HOME_POSITION);

            if(_drivetrain->GetNearTargetPosition() || _oi->IgnoreVision()){
               _auto_intake_algae_state = intake;
            }
            break;
        case intake:
            _intake->SetPower(IntakeConstants::EJECT_POWER);
            _funnel->SetPower(FunnelSubsystemConstants::INTAKE_POWER);

            if (_intake->AlgaeTop() && _intake->AlgaeBottom()) {
               _auto_intake_algae_state = done;
            }
            break;
        case done:
            _intake->SetPower(IntakeConstants::STOP_POWER);
            _funnel->SetPower(FunnelSubsystemConstants::STOP_POWER);
            break;
        default:
           _auto_intake_algae_state = wait;
            break;
    }
}

// Called once the command ends or is interrupted.
void TeleopIntakeAlgaeCommand::End(bool interrupted) {
    _intake->SetPower(IntakeConstants::STOP_POWER);
    _funnel->SetPower(FunnelSubsystemConstants::STOP_POWER);
}

// Returns true when the command should end.
bool TeleopIntakeAlgaeCommand::IsFinished() {
    return _auto_intake_algae_state == done;
}
