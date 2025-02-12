// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/teleop/auto/AutomaticIntakeCommand.h"

AutomaticIntakeCommand::AutomaticIntakeCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void AutomaticIntakeCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void AutomaticIntakeCommand::Execute() {}

// Called once the command ends or is interrupted.
void AutomaticIntakeCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool AutomaticIntakeCommand::IsFinished() {
    return false;
}
