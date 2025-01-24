// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SysIdRoutineBot.h"

#include <frc2/command/Commands.h>

SysIdRoutineBot::SysIdRoutineBot() {
    _ConfigureBindings();
}

void SysIdRoutineBot::_ConfigureBindings() {
    _m_pivot.SetDefaultCommand(_m_pivot.PsuedoSetAngle(
        [this] {return -_driver_controller.GetLeftY();}
    ));
}