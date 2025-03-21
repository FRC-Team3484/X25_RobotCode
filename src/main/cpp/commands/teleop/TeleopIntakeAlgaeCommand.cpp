#include "commands/teleop/TeleopIntakeAlgaeCommand.h"

#include <frc/smartdashboard/SmartDashboard.h>

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
    AddRequirements(_elevator);
    AddRequirements(_intake);
    AddRequirements(_pivot);
}

void TeleopIntakeAlgaeCommand::Initialize() {
    _auto_intake_algae_state = wait;
}

void TeleopIntakeAlgaeCommand::Execute() {
    switch(_auto_intake_algae_state) {
        case wait:
            // Wait unitl the robot is near the target position, then go to the next state
            //if(_drivetrain->GetNearTargetPosition() || _oi->IgnoreVision()){
            _auto_intake_algae_state = traveling_pivot;
            //}
            break;
        case traveling_pivot:
            // Set the pivot to the travel position
            // Once the pivot is at it's target position, go to the next state
            _pivot->SetPivotAngle(PivotConstants::TRAVEL_POSITION);
            if (_pivot->AtTargetPosition()) {
                _auto_intake_algae_state = extend_elevator;
            }
            break;
        case extend_elevator:
            // Set the height of the elevator, based off of the selected reef level
            // Once the elevator is at the target height and the drivetrain has reached the target position, go to the next state
           
            fmt::println("Extend Elevator");
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
            // Set the angle of the pivot
            // Once the pivot is at the target angle, go to the next state

            fmt::println("Extend Pivot");
            if (_oi->GetReefLevel() == 2 ||_oi->GetReefLevel() == 3){
                _pivot->SetPivotAngle(PivotConstants::TARGET_ALGAE_ANGLE);
            } 

            if(_pivot->AtTargetPosition()) {
                _auto_intake_algae_state = intake;
            }
            break;
        case intake:
            // Run the intake to intake an algae
            // Once the intake has the algae, go to the next state

            fmt::println("Intake");
            _intake->SetPower(IntakeConstants::INTAKE_POWER);

            //if (_intake->HasAlgae()) {
            //    _auto_intake_algae_state = done;
            //}
            break;
        case done:
            // Stop the intake

            fmt::println("Done");
            _intake->SetPower(IntakeConstants::STOP_POWER);
            break;
        default:
            _auto_intake_algae_state = wait;
            break;
    }
}

void TeleopIntakeAlgaeCommand::End(bool interrupted) {
    _intake->SetPower(IntakeConstants::STOP_POWER);
}

bool TeleopIntakeAlgaeCommand::IsFinished() {
    return _auto_intake_algae_state == done;
}
