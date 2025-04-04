// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/teleop/RumbleCommand.h"

RumbleCommand::RumbleCommand(Driver_Interface* driver_interface) : _driver_interface{driver_interface} {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void RumbleCommand::Initialize() {
  _driver_interface->SetRumble(SwerveConstants::ControllerConstants::DRIVER_RUMBLE_HIGH);
}

// Called repeatedly when this Command is scheduled to run
void RumbleCommand::Execute() {}

// Called once the command ends or is interrupted.
void RumbleCommand::End(bool interrupted) {
  _driver_interface->SetRumble(SwerveConstants::ControllerConstants::RUMBLE_STOP);
}

// Returns true when the command should end.
bool RumbleCommand::IsFinished() {
  return false;
}
