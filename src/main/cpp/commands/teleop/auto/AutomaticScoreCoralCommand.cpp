    // Copyright (c) FIRST and other WPILib contributors.
    // Open Source Software; you can modify and/or share it under the terms of
    // the WPILib BSD license file in the root directory of this project.

    #include "commands/teleop/auto/AutomaticScoreCoralCommand.h"

AutomaticScoreCoralCommand::AutomaticScoreCoralCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
}

    // Called when the command is initially scheduled.
void AutomaticScoreCoralCommand::Initialize() {}

    // Called repeatedly when this Command is scheduled to run
void AutomaticScoreCoralCommand::Execute() {
    switch(_auto_score_coral_state) {
        case wait:
            
            
            break;
        case extend_elevator:
            
            break;
        case extend_arm:
            
            break;
        case eject_piece:
            
            break;
        case retract_arm:
            
            break;
        case retract_elevator:
            
            break;
        default:
            _auto_score_coral_state = wait;
            break;
    }
}

    // Called once the command ends or is interrupted.
    void AutomaticScoreCoralCommand::End(bool interrupted) {}

    // Returns true when the command should end.
    bool AutomaticScoreCoralCommand::IsFinished() {
    return false;
    }
