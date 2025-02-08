// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#ifndef SYSIDSIDEWAYS_H
#define SYSIDSIDEWAYS_H

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include "Constants.h"
#include "subsystems/DrivetrainSubsystem.h"
#include "FRC3484_Lib/components/SC_Photon.h"

class SysIdSideways {
    public:
        SysIdSideways();

        frc2::CommandPtr GetAutonomousCommand();

    private:
    void _ConfigureBindings();
    frc2::CommandXboxController _driver_controller{UserInterface::Driver::DRIVER_CONTROLLER_PORT};

    SC_Photon* _vision_ptr = nullptr;
    DrivetrainSubsystem _drivetrain{};

    
    // Components (e.g. motor controllers and sensors) should generally be
    // declared private and exposed only through public methods.
};

#endif