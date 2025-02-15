// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

// Called when the command is initially scheduled.
void AutonFeederCoralCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void AutonFeederCoralCommand::Execute() {
   switch(_auton_feeder_coral_state) {
        case wait:
            _elevator->SetHeight(ElevatorConstants::HOME_POSITION);
            _pivot->SetPivotAngle(PivotConstants::HOME_POSITION);

            if(_drivetrain->GetNearTargetPosition()){
               _auton_feeder_coral_state = intake;
            }
            break;
        case intake:
            _intake->SetPower(IntakeConstants::EJECT_POWER);
            _funnel->SetPower(FunnelSubsystemConstants::INTAKE_POWER);

            if (_intake->CoralHigh() && _intake->CoralLow()) {
               _auton_feeder_coral_state = done;
            }
            break;
        case done:
            _intake->SetPower(IntakeConstants::STOP_POWER);
            _funnel->SetPower(FunnelSubsystemConstants::STOP_POWER);
            break;
        default:
           _auton_feeder_coral_state = wait;
            break;
    }
}

// Called once the command ends or is interrupted.
void AutonFeederCoralCommand::End(bool interrupted) {
    _intake->SetPower(IntakeConstants::STOP_POWER);
}

// Returns true when the command should end.
bool AutonFeederCoralCommand::IsFinished() {
    return _auton_feeder_coral_state == done;
}
