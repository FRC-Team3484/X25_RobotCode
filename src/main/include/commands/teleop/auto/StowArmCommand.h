#ifndef STOW_ARM_COMMAND_H
#define STOW_ARM_COMMAND_H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/PivotSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"

#include "OI.h"
#include "Constants.h"

/**
 * Moves the pivot arm to the stow/home position and lowers the elevator
 */
class StowArmCommand
    : public frc2::CommandHelper<frc2::Command, StowArmCommand> {

    public:
        StowArmCommand(PivotSubsystem* pivot_subsystem, ElevatorSubsystem* elevator_subsystem);

        void Initialize() override;
        void Execute() override;
        void End(bool interrupted) override;
        bool IsFinished() override;

    private:
        PivotSubsystem* _pivot_subsystem;
        ElevatorSubsystem* _elevator_subsystem;
};

#endif