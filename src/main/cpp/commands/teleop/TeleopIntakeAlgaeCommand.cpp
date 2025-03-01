#include "commands/teleop/TeleopIntakeAlgaeCommand.h"

TeleopIntakeAlgaeCommand::TeleopIntakeAlgaeCommand(
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
void TeleopIntakeAlgaeCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void TeleopIntakeAlgaeCommand::Execute() {
    switch(_auto_intake_algae_state) {
        case wait:
            if(_drivetrain->GetNearTargetPosition() || _oi->IgnoreVision()){
               _auto_intake_algae_state = traveling_pivot;
            }
            break;
        case traveling_pivot:
            _pivot->SetPivotAngle(PivotConstants::TRAVEL_POSITION);
            if (_pivot->AtTargetPosition()) {
                _auto_intake_algae_state = extend_elevator;
            }
            break;
        case extend_elevator:
            if (_oi->GetReefLevel() == 2) {
                _elevator->SetHeight(ElevatorConstants::ALGAE_LEVEL_2);
            } else if (_oi->GetReefLevel() == 3) {
                _elevator->SetHeight(ElevatorConstants::ALGAE_LEVEL_3);
            }

            if(_elevator->AtTargetHeight() && _drivetrain->GetAtTargetPosition()) {
                _auto_intake_algae_state = extend_pivot;
            }
            break;
        case extend_pivot:
            if (_oi->GetReefLevel() == 2 ||_oi->GetReefLevel() == 3){
                _pivot->SetPivotAngle(PivotConstants::TARGET_ALGAE_ANGLE);
            } 

            if(_pivot->AtTargetPosition()) {
                _auto_intake_algae_state = intake;
            }
            break;
        case intake:
            _intake->SetPower(IntakeConstants::INTAKE_POWER);

            if (_intake->HasAlgae()) {
               _auto_intake_algae_state = done;
            }
            break;
        case done:
            _intake->SetPower(IntakeConstants::STOP_POWER);
            break;
        default:
           _auto_intake_algae_state = wait;
            break;
    }
}

// Called once the command ends or is interrupted.
void TeleopIntakeAlgaeCommand::End(bool interrupted) {
    _intake->SetPower(IntakeConstants::STOP_POWER);
}

// Returns true when the command should end.
bool TeleopIntakeAlgaeCommand::IsFinished() {
    return _auto_intake_algae_state == done;
}
