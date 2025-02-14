// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/teleop/auto/AutomaticIntakeCommand.h"

AutomaticIntakeCommand::AutomaticIntakeCommand(
    DrivetrainSubsystem* drivetrain, 
    ElevatorSubsystem* elevator, 
    IntakeSubsystem* intake, 
    PivotSubsystem* pivot, 
    FunnelSubsystem* funnel, 
    Driver_Interface* oi) : 
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
void AutomaticIntakeCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void AutomaticIntakeCommand::Execute() {
    switch(_auto_intake_state) {
        case wait:
            _elevator->SetHeight(ElevatorConstants::HOME_POSITION);
            _pivot->SetPivotAngle(PivotConstants::HOME_POSITION);

            if(_drivetrain->GetNearTargetPosition()){
                _auto_intake_state = intake;
            }
            break;
        case intake:
            _intake->SetPower(IntakeConstants::EJECT_POWER);
            _funnel->SetPower(FunnelSubsystemConstants::INTAKE_POWER);

            if (_intake->CoralHigh() && _intake->CoralLow()) {
                _auto_intake_state = done;
            }
            break;
        case done:
            _intake->SetPower(IntakeConstants::STOP_POWER);
            _funnel->SetPower(FunnelSubsystemConstants::STOP_POWER);
            break;
        default:
            _auto_intake_state = wait;
            break;
    }
}

// Called once the command ends or is interrupted.
void AutomaticIntakeCommand::End(bool interrupted) {
    _intake->SetPower(IntakeConstants::STOP_POWER);
    _funnel->SetPower(FunnelSubsystemConstants::STOP_POWER);
}

// Returns true when the command should end.
bool AutomaticIntakeCommand::IsFinished() {
    return _auto_intake_state == done;
}
