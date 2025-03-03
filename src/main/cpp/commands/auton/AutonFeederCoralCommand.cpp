#include "commands/auton/AutonFeederCoralCommand.h"

AutonFeederCoralCommand::AutonFeederCoralCommand(
    DrivetrainSubsystem* drivetrain, 
    ElevatorSubsystem* elevator, 
    IntakeSubsystem* intake, 
    PivotSubsystem* pivot, 
    FunnelSubsystem* funnel) : 
    _drivetrain(drivetrain),
    _elevator(elevator),
    _intake(intake),
    _pivot(pivot),
    _funnel(funnel) {
    AddRequirements(_drivetrain);
    AddRequirements(_elevator);
    AddRequirements(_intake);
    AddRequirements(_pivot);
    AddRequirements(_funnel);
}

void AutonFeederCoralCommand::Initialize() {}

void AutonFeederCoralCommand::Execute() {
    switch(_auton_feeder_coral_state) {
        case wait:
            // Stow the elevator and pivot
            // Once the robot is near the target position, go to next state
            _elevator->SetHeight(ElevatorConstants::HOME_POSITION);
            _pivot->SetPivotAngle(PivotConstants::HOME_POSITION);
            if(_drivetrain->GetNearTargetPosition()){
                _auton_feeder_coral_state = intake;
            }
            break;
        case intake:
            // Run the intake and funnel to feed in coral
            // Once coral is in the intake, go to next state
            _intake->SetPower(IntakeConstants::EJECT_POWER);
            if (_funnel != nullptr) _funnel->SetPower(FunnelConstants::INTAKE_POWER);
            if (_intake->HasCoral()) {
                _auton_feeder_coral_state = done;
            }
            break;
        case done:
            // Stop the intake and funnel
            _intake->SetPower(IntakeConstants::STOP_POWER);
            if (_funnel != nullptr) _funnel->SetPower(FunnelConstants::STOP_POWER);
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
