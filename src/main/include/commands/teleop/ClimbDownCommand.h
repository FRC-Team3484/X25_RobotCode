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
class ClimbDownCommand
    : public frc2::CommandHelper<frc2::Command, ClimbDownCommand> {

    public:
        /**
         * Creates an instnce of TeleopDriveCommand
         * 
         * @param elevator A pointer to the elevator interface
         */
        ClimbDownCommand(ElevatorSubsystem* elevator);

        void Initialize() override;
        void Execute() override;
        void End(bool interrupted) override;
        bool IsFinished() override;

    private:
        enum State {climb_down_elevator, done};
        State _climb_down_state = climb_down_elevator;

        ElevatorSubsystem* _elevator;
};

#endif