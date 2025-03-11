#include "commands/teleop/TeleopIntakeCoralCommand.h"

TeleopIntakeCoralCommand::TeleopIntakeCoralCommand(
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

void TeleopIntakeCoralCommand::Initialize() {
    _auto_intake_coral_state = wait;
}

void TeleopIntakeCoralCommand::Execute() {
    switch(_auto_intake_coral_state) {
        case wait:
            // Wait until the robot is near the target position, then go to the next state
            if(_drivetrain->GetNearTargetPosition() || _oi->IgnoreVision()){
                _auto_intake_coral_state = ready_intake;
            }
            break;
        case ready_intake:
            // Set the pivot to the intake position
            // Once the pivot is at it's target position, go to the next state
            _pivot->SetPivotAngle(PivotConstants::INTAKE_POSITION);
            if (_pivot->AtTargetPosition()) {
                _auto_intake_coral_state = intake;
            }
            break;
        case intake:
            // Run the intake
            // Once the intake has coral, go to the next state
            _intake->SetPower(IntakeConstants::INTAKE_POWER);
            if (_funnel != nullptr) _funnel->SetPower(FunnelConstants::INTAKE_POWER);

            if (_intake->CoralLow()) {
                _auto_intake_coral_state = done;
            }
            break;
        case done:
            // Stop the intake and funnel
            _intake->SetPower(IntakeConstants::STOP_POWER);
            if (_funnel != nullptr) _funnel->SetPower(FunnelConstants::STOP_POWER);
            break;
        default:
            _auto_intake_coral_state = wait;
            break;
    }
}

void TeleopIntakeCoralCommand::End(bool interrupted) {
    _intake->SetPower(IntakeConstants::STOP_POWER);
    if (_funnel != nullptr) _funnel->SetPower(FunnelConstants::STOP_POWER);
}

bool TeleopIntakeCoralCommand::IsFinished() {
    return _auto_intake_coral_state == done;
}
