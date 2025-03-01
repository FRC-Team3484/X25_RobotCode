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
        /**
         * Creates an instnce of TeleopDriveCommand
         * 
         * @param pivot A pointer to the pivot subsystem
         * @param elevator A pointer to the elevator interface
         */
        StowArmCommand(PivotSubsystem* pivot, ElevatorSubsystem* elevator);

        void Initialize() override;
        void Execute() override;
        void End(bool interrupted) override;
        bool IsFinished() override;

    private:
        enum State {traveling_pivot, stow_arm, stow_elevator, done};
        State _stow_arm_state = traveling_pivot;

        PivotSubsystem* _pivot;
        ElevatorSubsystem* _elevator;
};

#endif