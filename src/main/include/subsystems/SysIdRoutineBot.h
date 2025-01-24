    // Copyright (c) FIRST and other WPILib contributors.
    // Open Source Software; you can modify and/or share it under the terms of
    // the WPILib BSD license file in the root directory of this project.

#ifndef SYSIDROUTINEBOT_H
#define SYSIDROUTINEBOT_H

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.h"
#include "subsystems/ElevatorSubsystem.h"


class SysIdRoutineBot {
    public:
        SysIdRoutineBot();

        frc2::CommandPtr GetAutonomousCommand();

    private:
        void _ConfigureBindings();
        frc2::CommandXboxController _driver_controller{UserInterface::Driver::DRIVER_CONTROLLER_PORT};
        
        ElevatorSubsystem _m_elevator{
            Elevator::PRIMARY_MOTOR_CAN_ID,
            Elevator::SECONDARY_MOTOR_CAN_ID,
            Elevator::HOME_SENSOR_DI_CH,
            Elevator::,
            Elevator::MAX_VELOCITY, Elevator::MAX_ACCELERATION, Elevator::ELEVATOR_FEED_FORWARD_CONSTANTS};
};

#endif