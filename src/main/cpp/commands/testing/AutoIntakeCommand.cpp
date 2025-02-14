// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/testing/AutoIntakeCommand.h"

AutoIntakeCommand::AutoIntakeCommand() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void AutoIntakeCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void AutoIntakeCommand::Execute() {}

// Called once the command ends or is interrupted.
void AutoIntakeCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool AutoIntakeCommand::IsFinished() {
  return false;
}
