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
     * Creates the test intake command
     * 
     * @param intake_subsystem The intake subsystem
     * @param oi The testing interface from OI.h
     */
    TestIntakeCommand(IntakeSubsystem *intake_subsystem, Testing_Interface *oi);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

private:
    IntakeSubsystem *_intake_subsystem;
    Testing_Interface *_oi;
};

#endif