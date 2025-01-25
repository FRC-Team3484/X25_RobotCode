// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SysIdForward.h"

#include <frc2/command/Commands.h>

SysIdForward::SysIdForward() {
    _ConfigureBindings();
}

void SysIdForward::_ConfigureBindings() {

}
// This method will be called once per scheduler run

frc2::CommandPtr SysIdForward::GetAutonomousCommand() {
    return _drivetrain.Run([] {});
}