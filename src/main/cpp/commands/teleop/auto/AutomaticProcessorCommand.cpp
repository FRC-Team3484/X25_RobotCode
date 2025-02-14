// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/teleop/auto/AutomaticProcessorCommand.h"
#include "Constants.h"

AutomaticProcessorCommand::AutomaticProcessorCommand(
    ElevatorSubsystem* elevator_subsystem,
    PivotSubsystem* pivot_subsystem,
    DrivetrainSubsystem* drivetrain_subsystem,
    IntakeSubsystem* intake_subsystem,
    Operator_Interface* oi
    ) :
    _elevator_subsystem{elevator_subsystem},
    _pivot_subsystem{pivot_subsystem},
    _drivetrain_subsystem{drivetrain_subsystem},
    _intake_subsystem{intake_subsystem},
    _oi{oi}
    {
    
    AddRequirements(_elevator_subsystem);
    AddRequirements(_pivot_subsystem);
    AddRequirements(_drivetrain_subsystem);
    AddRequirements(_intake_subsystem);
}

// Called when the command is initially scheduled.
void AutomaticProcessorCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void AutomaticProcessorCommand::Execute() {
    switch (_auto_processor_state) {
        case wait:
            // if drivetrain is near target position, got to extend_elevator.
            if (_drivetrain_subsystem->GetNearTargetPosition()) {
                _auto_processor_state = extend_elevator;
            }
            break; 
        case extend_elevator:
            // extend elevator to score in the processor
            // if elevator is at target height, go to extend_pivot
            _elevator_subsystem->SetHeight(ElevatorConstants::PROCESSOR_POSITION_1);
            if (_elevator_subsystem->AtTargetHeight()) {
                _auto_processor_state = extend_pivot;
            }
            break;
        case extend_pivot:
            // set pivot for scoring in the processor
            // if pivot is at target angle, go to eject_algae
            _pivot_subsystem->SetPivotAngle(PivotConstants::TARGET_POSITION);
            if (_pivot_subsystem->AtTargetPosition()) {
                _auto_processor_state = eject_algae;
            }
            break;
        case eject_algae:
            // eject algae from intake
            // when intake doesn't detect a piece, go to retract_pivot
            _intake_subsystem->SetPower(IntakeConstants::EJECT_POWER);
            if (_intake_subsystem->AlgaeTop()) {
                _auto_processor_state = retract_pivot;
            }
            break;
        // case retract_pivot:
        //     // set pivot angle to home position
        //     // if pivot is at home position, go to retract_elevator
        //     _pivot_subsystem->SetPivotAngle(PivotConstants::HOME_POSITION);
        //     if (_pivot_subsystem->AtTargetPosition()) {
        //         _auto_processor_state = retract_elevator;
        //     }
        //     break;
        // case retract_elevator:
        //     // set elevator to home position
        //     _elevator_subsystem->SetHeight(ElevatorConstants::HOME_POSITION);
        //     break;
        default:
            _auto_processor_state = wait;
            break;
    }
}

// Called once the command ends or is interrupted.
void AutomaticProcessorCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool AutomaticProcessorCommand::IsFinished() {
    return false;
}
