// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SysIdRoutineBot.h"

#include <frc2/command/Commands.h>

SysIdRoutineBot::SysIdRoutineBot() {
    _ConfigureBindings();
}

void SysIdRoutineBot::_ConfigureBindings() {
    _elevator.SetDefaultCommand(_elevator.PsuedoSetPower(
        [this] {return -_driver_controller.GetLeftY();}
    ));
    
    (_driver_controller.A())
        .WhileTrue(_elevator.SysIdQuasistatic(frc2::sysid::Direction::kForward));
    (_driver_controller.B())
        .WhileTrue(_elevator.SysIdQuasistatic(frc2::sysid::Direction::kReverse));
    (_driver_controller.X())
        .WhileTrue(_elevator.SysIdDynamic(frc2::sysid::Direction::kForward));
    (_driver_controller.Y())
        .WhileTrue(_elevator.SysIdDynamic(frc2::sysid::Direction::kReverse));

}