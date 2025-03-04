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

void AutonScoreCoralCommand::Initialize() {}

void AutonScoreCoralCommand::Execute() {
    switch(_auton_score_coral_state) {
        case wait:
            // Wait until the robot is near the target position, then go to next state
            if(_drivetrain->GetNearTargetPosition()) {
                _auton_score_coral_state = extend_elevator;
            }
            break;
        case extend_elevator:
            // Set the height of the elevator
            // Once the elevator is at the target height, go to the next state

            //_elevator->SetHeight(ElevatorConstants::PROCESSOR_POSITION); FIX LATER ONCE WE ACTUALLY HAVE AUTONS
            if(_elevator->AtTargetHeight() && _drivetrain->GetAtTargetPosition()) {
                _auton_score_coral_state = extend_pivot;
            }
            break;
        case extend_pivot:
            // Set the angle of the pivot
            // Once the pivot is at the target angle, go to the next state

            //_pivot->SetPivotAngle(PivotConstants::TARGET_POSITION); 
            if(_pivot->AtTargetPosition()) {
                _auton_score_coral_state = eject_piece;
            }
            break;
        case eject_piece:
            // Run the intake to eject the piece
            // Once the intake no longer has coral, go to the next state
            _intake->SetPower(IntakeConstants::EJECT_POWER);
            if(!_intake->HasCoral()) {
                _auton_score_coral_state = done;
            }
            break;
        case done:
            // Stop the intake
            _intake->SetPower(IntakeConstants::STOP_POWER);
            break;
        default:
            _auton_score_coral_state = wait;
            break;
    }
}

void AutonScoreCoralCommand::End(bool interrupted) {
    _intake->SetPower(IntakeConstants::STOP_POWER);
}

bool AutonScoreCoralCommand::IsFinished() {
    return _auton_score_coral_state == done;
}
