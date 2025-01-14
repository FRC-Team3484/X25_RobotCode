// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#ifndef OI_H
#define OI_H

#include "Constants.h"
#include <frc/XboxController.h>

class Driver_Interface {
    public:
        Driver_Interface();
        //  Swerve Controllers
        double GetThrottle();
        double GetStrafe();
        double GetRotation();

        bool GetResetHeading();
        bool GetBrake();
        bool GetBrakePressed();

        bool GetSetBrakeMode();
        bool GetDisableBrakeMode();

        bool LowSpeed();
        void SetRumble(double Rumble);

        bool DriverOverride();

    private:
        frc::XboxController _driver_controller{UserInterface::Driver::DRIVER_CONTROLLER_PORT};
};

class Operator_Interface {
    public:
        Operator_Interface();

        void SetRumble(double Rumble);

        int RawPOV();

    private:
        frc::XboxController _operator_controller{UserInterface::Operator::OPERATOR_CONTROLLER_PORT};
};

#endif