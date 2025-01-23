// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "commands/testing/TestPivotCommand.h"
#include "frc/smartdashboard/SmartDashboard.h"


TestPivotCommand::TestPivotCommand(PivotSubsystem* pivot_subsystem, Testing_Interface* testing_interface)
: _pivot_subsystem(pivot_subsystem),_testing_interface(testing_interface) {
  AddRequirements({_pivot_subsystem});
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void TestPivotCommand::Initialize() {
  _pivot_subsystem->SetTestMode(true);
  frc::SmartDashboard::PutBoolean("Test Pivot", true);
}

// Called repeatedly when this Command is scheduled to run
void TestPivotCommand::Execute() {
  if (frc::SmartDashboard::GetBoolean("Test Pivot", false)){
    _pivot_subsystem->SetPower(_testing_interface->GetMotorOne());
    _pivot_subsystem->PrintTestInfo();
  }
}

// Called once the command ends or is interrupted.
void TestPivotCommand::End(bool interrupted) {
  _pivot_subsystem->SetTestMode(false);
}

// Returns true when the command should end.
bool TestPivotCommand::IsFinished() {
  return false;
}
