#ifndef CLIMB_UP_COMMAND_H
#define CLIMB_UP_COMMAND_H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/PivotSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"

#include "OI.h"
#include "Constants.h"

/**
 * Moves the pivot arm to the stow/home position and lowers the elevator
 */
class ClimbUpCommand
    : public frc2::CommandHelper<frc2::Command, ClimbUpCommand> {

    public:
        /**
         * Creates an instnce of TeleopDriveCommand
         * 
         * @param elevator A pointer to the elevator interface
         */
        ClimbUpCommand(ElevatorSubsystem* elevator, PivotSubsystem* pivot);

        void Initialize() override;
        void Execute() override;
        void End(bool interrupted) override;
        bool IsFinished() override;

    private:
        enum State {traveling_pivot, climb_up_elevator, done};
        State _climb_up_state = traveling_pivot;

        ElevatorSubsystem* _elevator;
        PivotSubsystem* _pivot;
};

#endif