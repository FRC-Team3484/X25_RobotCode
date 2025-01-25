#ifndef AUTONSTOPCOMMAND_H
#define AUTONSTOPCOMMAND_H

#include "OI.h"
#include "Constants.h"
#include "subsystems/DrivetrainSubsystem.h"

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

class AutonStopCommand
    :public frc2::CommandHelper<frc2::Command, AutonStopCommand> {

    public:
        explicit AutonStopCommand(DrivetrainSubsystem* drivetrain_subsystem);

        void Initialize() override;
        void Execute() override;
        void End(bool inturrupted) override;
        bool IsFinished() override;

    private:
        DrivetrainSubsystem* _drivetrain_subsystem;

};

#endif