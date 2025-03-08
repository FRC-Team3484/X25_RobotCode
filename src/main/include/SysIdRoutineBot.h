
#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>


#include "Constants.h"
#include "OI.h"
#include "subsystems/DrivetrainSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"
#include "subsystems/PivotSubsystem.h"

class SysIdRoutineBot {
    public:
        SysIdRoutineBot(DrivetrainSubsystem* drive, ElevatorSubsystem* elevator, PivotSubsystem* pivot);

        frc2::Command* GetAutonomousCommand();
    private:
        void ConfigureBindings();
        frc::XboxController _sysid_driver_testing_controller{UserInterface::Testing::TESTING_CONTROLLER_PORT};

        DrivetrainSubsystem* _drive;
        ElevatorSubsystem* _elevator;
        PivotSubsystem* _pivot;
        frc::EventLoop* _event_loop;
};