// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#ifndef AUTOMATIC_INTAKE_COMMAND_H
#define AUTOMATIC_INTAKE_COMMAND_H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/IntakeSubsystem.h"
#include "subsystems/PivotSubsystem.h"
#include "subsystems/DrivetrainSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"
#include "subsystems/FunnelSubsystem.h"

#include "OI.h"

#include "StowArmCommand.h"

/**
* An example command.
*
* <p>Note that this extends CommandHelper, rather extending Command
* directly; this is crucially important, or else the decorator functions in
* Command will *not* work!
*/
class AutomaticIntakeCommand
    : public frc2::CommandHelper<frc2::Command, AutomaticIntakeCommand>
    {
    
    public:
        /* You should consider using the more terse Command factories API instead
        * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
        */
        AutomaticIntakeCommand();

        void Initialize() override;

        void Execute() override;

        void End(bool interrupted) override;

        bool IsFinished() override;

    private:
        bool _isFinished = false;
        

        enum State {wait, intake, done};
        State _auto_intake_state = wait;
};

#endif