#include "commands/auton/AutonFeederCoralCommand.h"

AutonFeederCoralCommand::AutonFeederCoralCommand(
    DrivetrainSubsystem* drivetrain, 
    ElevatorSubsystem* elevator,
    IntakeSubsystem* intake, 
    PivotSubsystem* pivot) : 
    _drivetrain(drivetrain), 
    _elevator(elevator),
    _intake(intake),
    _pivot(pivot) {
    AddRequirements(_elevator);
    AddRequirements(_intake);
    AddRequirements(_pivot);
}

void AutonFeederCoralCommand::Initialize() {}

void AutonFeederCoralCommand::Execute() {
    switch(_auton_feeder_coral_state) {
        case wait:
            // Stow the elevator and pivot
            // Once the robot is near the target position, go to next state
            _elevator->SetHeight(ElevatorConstants::INTAKE_HEIGHT);
            _pivot->SetPivotAngle(PivotConstants::INTAKE_POSITION);
            if(_pivot->AtTargetPosition()){
                _auton_feeder_coral_state = intake;
            }
            break;
        case intake:
            // Run the intake and funnel to feed in coral
            // Once coral is in the intake, go to next state
            _intake->SetPower(IntakeConstants::CORAL_EJECT_POWER);
            if (_intake->CoralLow()) {
                _auton_feeder_coral_state = done;
            }
            break;
        case done:
            // Stop the intake and funnel
            _intake->SetPower(IntakeConstants::STOP_POWER);
            break;
        default:
            _auton_feeder_coral_state = wait;
            break;
    }
}

void AutonFeederCoralCommand::End(bool interrupted) {
    _intake->SetPower(IntakeConstants::STOP_POWER);
}

bool AutonFeederCoralCommand::IsFinished() {
    return _auton_feeder_coral_state == done;
}
