#ifndef TEST_INTAKE_SUBSYSTEM_H
#define TEST_INTAKE_SUBSYSTEM_H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/IntakeSubsystem.h"
#include "OI.h"

/**
 * Test Intake Command
 * 
 * This command is run to test the functions of the intake
 */
class TestIntakeCommand
    : public frc2::CommandHelper<frc2::Command, TestIntakeCommand>
{
    public:
        /**
         * Allows the intake to be tested by using the controller buttons
         * 
         * @param intake_subsystem A pointer to the intake subsystem
         * @param testing_interface A pointer to the testing interface
         */
        TestIntakeCommand(IntakeSubsystem *intake_subsystem, Testing_Interface *testing_interface);

        void Initialize() override;

        void Execute() override;

        void End(bool interrupted) override;

        bool IsFinished() override;

    private:
        IntakeSubsystem *_intake_subsystem;
        Testing_Interface *_testing_interface;
};

#endif