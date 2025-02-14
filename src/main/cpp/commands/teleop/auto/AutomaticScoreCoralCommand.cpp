    // Copyright (c) FIRST and other WPILib contributors.
    // Open Source Software; you can modify and/or share it under the terms of
    // the WPILib BSD license file in the root directory of this project.

    #include "commands/teleop/auto/AutomaticScoreCoralCommand.h"
    #include "Constants.h"

AutomaticScoreCoralCommand::AutomaticScoreCoralCommand(
    IntakeSubsystem* intake_subsystem, 
    PivotSubsystem* pivot_subsystem, 
    DrivetrainSubsystem* drivetrain_subsystem, 
    ElevatorSubsystem* elevator_subsystem, 
    Driver_Interface* oi) 
    // Use addRequirements() here to declare subsystem dependencies.
    : _drivetrain_subsystem{drivetrain_subsystem}, 
    _pivot_subsystem{pivot_subsystem},
    _intake_subsystem{intake_subsystem},
    _elevator_subsystem{elevator_subsystem}, 
    _oi(oi)  {
    AddRequirements(_drivetrain_subsystem);
    AddRequirements(_pivot_subsystem);
    AddRequirements(_elevator_subsystem);
}

    // Called when the command is initially scheduled.
void AutomaticScoreCoralCommand::Initialize() {

}

    // Called repeatedly when this Command is scheduled to run
void AutomaticScoreCoralCommand::Execute() {
    switch(_auto_score_coral_state) {
        case wait:
            if(_drivetrain_subsystem->GetNearTargetPosition()) {
                _auto_score_coral_state = extend_elevator;
            }
            break;
        case extend_elevator:
            _elevator_subsystem->SetHeight(ElevatorConstants::PROCESSOR_POSITION_3);

            if(_elevator_subsystem->AtTargetHeight()) {
                _auto_score_coral_state = extend_pivot;
            }
            break;
        case extend_pivot:
            _pivot_subsystem->SetPivotAngle(PivotConstants::TARGET_POSITION);

            if(_pivot_subsystem->AtTargetPosition()) {
                _auto_score_coral_state = eject_piece;
            }
            break;
        case eject_piece:
            _intake_subsystem->SetPower(IntakeConstants::EJECT_POWER);

            if(!_intake_subsystem->CoralHigh() && !_intake_subsystem->CoralLow()) {
                _auto_score_coral_state = retract_pivot;
            }
            break;
        default:
            _auto_score_coral_state = wait;
            break;
    }
}

    // Called once the command ends or is interrupted.
    void AutomaticScoreCoralCommand::End(bool interrupted) {
        _drivetrain_subsystem->StopMotors();
        _intake_subsystem->SetPower(0);

    }

    // Returns true when the command should end.
    bool AutomaticScoreCoralCommand::IsFinished() {
    return false;
    }
