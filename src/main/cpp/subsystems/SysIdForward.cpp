// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SysIdForward.h"

#include <frc2/command/Commands.h>

SysIdForward::SysIdForward() {
    _ConfigureBindings();
}

void SysIdForward::_ConfigureBindings() {
    _drivetrain.SetDefaultCommand(_drivetrain.PseudoForwardCommand(
      [this] { return -_driver_controller.GetLeftY(); }));

  // Using bumpers as a modifier and combining it with the buttons so that we
  // can have both sets of bindings at once
    (_driver_controller.A())
      .WhileTrue(_drivetrain.SysIdForwardQuasistatic(frc2::sysid::Direction::kForward));
    (_driver_controller.B())
      .WhileTrue(_drivetrain.SysIdForwardQuasistatic(frc2::sysid::Direction::kReverse));
    (_driver_controller.X())
      .WhileTrue(_drivetrain.SysIdForwardDynamic(frc2::sysid::Direction::kForward));
    (_driver_controller.Y())
      .WhileTrue(_drivetrain.SysIdForwardDynamic(frc2::sysid::Direction::kReverse));
}
// This method will be called once per scheduler run

frc2::CommandPtr SysIdForward::GetAutonomousCommand() {
    return _drivetrain.Run([] {});
}