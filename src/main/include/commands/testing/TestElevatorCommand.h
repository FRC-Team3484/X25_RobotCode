#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/ElevatorSubsystem.h"
#include "OI.h"

class TestElevatorCommand
    : public frc2::CommandHelper<frc2::Command, TestElevatorCommand> {
    public:
        
        /**
         * Allows the elevator to be tested using the controller buttons
         * 
         * @param elevator_subsystem A pointer to the elevator subsystem
         * @param testing_interface A pointer to the testing interface
         */
        TestElevatorCommand(
            ElevatorSubsystem* elevator_subsystem,
            Testing_Interface* testing_interface 
        );

        void Initialize() override;

        void Execute() override;

        void End(bool interrupted) override;

        bool IsFinished() override;
    private:
        ElevatorSubsystem* _elevator_subsystem;
        Testing_Interface* _testing_interface;

};
