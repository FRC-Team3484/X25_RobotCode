// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/teleop/auto/AutomaticProcessorCommand.h"
#include "Constants.h"

AutomaticProcessorCommand::AutomaticProcessorCommand(
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
void AutomaticProcessorCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void AutomaticProcessorCommand::Execute() {
    switch (_auto_processor_state) {
        case wait:
            // if drivetrain is near target position, got to extend_elevator.
            if (_drivetrain->GetNearTargetPosition()) {
                _auto_processor_state = extend_elevator;
            }
            break; 
        case extend_elevator:
            // extend elevator to score in the processor
            // if elevator is at target height, go to extend_pivot
            _elevator->SetHeight(ElevatorConstants::PROCESSOR_POSITION_1);
            if (_elevator->AtTargetHeight()) {
                _auto_processor_state = extend_pivot;
            }
            break;
        case extend_pivot:
            // set pivot for scoring in the processor
            // if pivot is at target angle, go to eject_algae
            _pivot->SetPivotAngle(PivotConstants::TARGET_POSITION);
            if (_pivot->AtTargetPosition()) {
                _auto_processor_state = eject_algae;
            }
            break;
        case eject_algae:
            // eject algae from intake
            // when intake doesn't detect a piece, go to retract_pivot
            _intake->SetPower(IntakeConstants::EJECT_POWER);
            if (_intake->AlgaeTop()) {
                _auto_processor_state = done;
            }
            break;
        case done:
            // stop intake
            _intake->SetPower(IntakeConstants::STOP_POWER);
            break;
        default:
            _auto_processor_state = wait;
            break;
    }
}

// Called once the command ends or is interrupted.
void AutomaticProcessorCommand::End(bool interrupted) {
    _intake->SetPower(IntakeConstants::STOP_POWER);
}

// Returns true when the command should end.
bool AutomaticProcessorCommand::IsFinished() {
    return _auto_processor_state == done;
}
