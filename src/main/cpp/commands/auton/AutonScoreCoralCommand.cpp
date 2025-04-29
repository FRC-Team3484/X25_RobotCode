#include "commands/auton/AutonScoreCoralCommand.h"
#include "Constants.h"

AutonScoreCoralCommand::AutonScoreCoralCommand(
    DrivetrainSubsystem* drivetrain, 
    ElevatorSubsystem* elevator,
    IntakeSubsystem* intake, 
    PivotSubsystem* pivot,
    AutonLevel::AutonLevel reef_level) : 
    _drivetrain(drivetrain), 
    _elevator(elevator),
    _intake(intake),
    _pivot(pivot),
    _reef_level(reef_level) {
    AddRequirements(_elevator);
    AddRequirements(_intake);
    AddRequirements(_pivot);
}

void AutonScoreCoralCommand::Initialize() {
    _auton_score_coral_state = wait;
    _eject_timer.Stop();
    _eject_timer.Reset();
}

void AutonScoreCoralCommand::Execute() {
    switch(_auton_score_coral_state) {
        case wait:
            // Wait until the robot is near the target position, then go to next state
            if(_drivetrain->GetNearTargetPosition()) {
                _auton_score_coral_state = traveling_pivot;
            }
            break;
        case traveling_pivot:
            _pivot->SetPivotAngle(PivotConstants::TRAVEL_POSITION);
            if (_pivot->AtTargetPosition()) {
                _auton_score_coral_state = extend_elevator;
            }
            break;
        case extend_elevator:
            if (_reef_level == AutonLevel::level_1) {
                _elevator->SetHeight(ElevatorConstants::CORAL_LEVEL_1);
            } else if (_reef_level == AutonLevel::level_2) {
                _elevator->SetHeight(ElevatorConstants::CORAL_LEVEL_2);
            } else if (_reef_level == AutonLevel::level_3) {
                _elevator->SetHeight(ElevatorConstants::CORAL_LEVEL_3);
            } else if (_reef_level == AutonLevel::level_4) {
                _elevator->SetHeight(ElevatorConstants::CORAL_LEVEL_4);
            }

            if(_elevator->AtTargetHeight() && _drivetrain->GetAtTargetPosition()) {
                _auton_score_coral_state = extend_pivot;
            }
            break;
        case extend_pivot:
            if ((_reef_level == AutonLevel::level_2) || (_reef_level == AutonLevel::level_3)) {
                _pivot->SetPivotAngle(PivotConstants::TARGET_CORAL_ANGLE);
            } else if (_reef_level == AutonLevel::level_4) {
                _pivot->SetPivotAngle(PivotConstants::TARGET_CORAL_4_ANGLE);
            } else if(_reef_level == AutonLevel::level_1){
                _pivot->SetPivotAngle(PivotConstants::TARGET_CORAL_1_ANGLE);
            }

            if(_pivot->AtTargetPosition()) {
                _auton_score_coral_state = eject_piece;
                _eject_timer.Start();
            }
            break;
        case eject_piece:
            _intake->SetPower(IntakeConstants::CORAL_EJECT_POWER);

            if (!_intake->HasCoral() && _eject_timer.HasElapsed(1_s)) {
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
    _eject_timer.Stop();
}

bool AutonScoreCoralCommand::IsFinished() {
    return _auton_score_coral_state == done;
}
