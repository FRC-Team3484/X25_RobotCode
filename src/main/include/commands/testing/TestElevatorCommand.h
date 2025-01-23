#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/ElevatorSubsystem.h"
#include "OI.h"

/**
 * Test Elevator Command
 * 
 * This command allows the elevator to be tested using the controller buttons
 */
class TestElevatorCommand
    : public frc2::CommandHelper<frc2::Command, TestElevatorCommand> {
    public:
        
        /**
         * Creates an instance of the test elevator command
         * 
         * @param elevator_subsystem The elevator subsystme needed for this command
         * @param testing_interface The testing interface from OI
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
