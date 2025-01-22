// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/testing/TestIntakeCommand.h"
#include <frc/smartdashboard/SmartDashboard.h>

TestIntakeCommand::TestIntakeCommand(IntakeSubsystem* intake_subsystem, Testing_Interface* oi)
:
  _intake_subsystem{intake_subsystem}, _oi{oi} {
    AddRequirements(_intake_subsystem);
}

// Called when the command is initially scheduled.
void TestIntakeCommand::Initialize() {
  frc::SmartDashboard::PutBoolean("Test Intake", false);
}

// Called repeatedly when this Command is scheduled to run
void TestIntakeCommand::Execute() {
  if (frc::SmartDashboard::GetBoolean("Test Intake", false)) {
    _intake_subsystem->PrintTestInfo();
    _intake_subsystem->SetRollerPower(_oi->GetMotor1());
  }
}

// Called once the command ends or is interrupted.
void TestIntakeCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool TestIntakeCommand::IsFinished() {
  return false;
}
