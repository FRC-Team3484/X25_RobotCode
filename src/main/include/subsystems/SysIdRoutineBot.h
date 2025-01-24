    // Copyright (c) FIRST and other WPILib contributors.
    // Open Source Software; you can modify and/or share it under the terms of
    // the WPILib BSD license file in the root directory of this project.

#ifndef SYSIDROUTINEBOT_H
#define SYSIDROUTINEBOT_H

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.h"
#include "subsystems/PivotSubsystem.h"


class SysIdRoutineBot {
    public:
        SysIdRoutineBot();

        frc2::CommandPtr GetAutonomousCommand();

    private:
        void _ConfigureBindings();
        frc2::CommandXboxController _driver_controller{UserInterface::Driver::DRIVER_CONTROLLER_PORT};
        
        PivotSubsystem _m_pivot{
            Pivot::PIVOT_MOTOR_CAN_ID,
            Pivot::PIVOT_HOME_DI_CH,
            Pivot::PID_C,
            Pivot::MAX_VELOCITY,
            Pivot::MAX_ACCELERATION,
            Pivot::FEED_FORWARD
            };

    

};

#endif