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

void TeleopProcessorCommand::Initialize() {}

void TeleopProcessorCommand::Execute() {
    switch (_auto_processor_state) {
        case wait:
            // Wait until the robot is near the target position, then go to the next state
            if (_drivetrain->GetNearTargetPosition() || _oi->IgnoreVision()) {
                _auto_processor_state = traveling_pivot;
            }
            break; 
        case traveling_pivot:
            // Set the pivot to the travel position
            // Once the pivot is at it's target position, go to the next state
            _pivot->SetPivotAngle(PivotConstants::TRAVEL_POSITION);
            if (_pivot->AtTargetPosition()) {
                _auto_processor_state = extend_elevator;
            }
            break;
        case extend_elevator:
            // Set the height of the elevator to the processor position
            // Once the elevator is at the target height and the drivetrain has reached the target position, go to the next state
            _elevator->SetHeight(ElevatorConstants::PROCESSOR_POSITION);
            if (_elevator->AtTargetHeight() && _drivetrain->GetAtTargetPosition()) {
                _auto_processor_state = extend_pivot;
            }
            break;
        case extend_pivot:
            // Set the angle of the pivot to the processor position
            // Once the pivot is at the target angle, go to the next state
            _pivot->SetPivotAngle(PivotConstants::PROCESSOR_POSITION);
            if (_pivot->AtTargetPosition()) {
                _auto_processor_state = eject_algae;
            }
            break;
        case eject_algae:
            // Run the intake to eject the algae
            // Once the intake no longer has algae, go to the next state
            _intake->SetPower(IntakeConstants::EJECT_POWER);
            if (!_intake->HasAlgae()) {
                _auto_processor_state = done;
            }
            break;
        case done:
            // Stop the intake
            _intake->SetPower(IntakeConstants::STOP_POWER);
            break;
        default:
            _auto_processor_state = wait;
            break;
    }
}

void TeleopProcessorCommand::End(bool interrupted) {
    _intake->SetPower(IntakeConstants::STOP_POWER);
}

bool TeleopProcessorCommand::IsFinished() {
    return _auto_processor_state == done;
}
