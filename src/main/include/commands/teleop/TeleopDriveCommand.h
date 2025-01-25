#ifndef DRIVE_COMMAND_H
#define DRIVE_COMMAND_H

#include "OI.h"
#include "subsystems/DrivetrainSubsystem.h"

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/DriverStation.h>

class TeleopDriveCommand: public frc2::CommandHelper<frc2::Command, TeleopDriveCommand> {
    public:
        explicit TeleopDriveCommand(DrivetrainSubsystem* drivetrain, Driver_Interface* oi);

        void Initialize() override;
        void Execute() override;
        void End(bool interrupted) override;
        bool IsFinished() override; 

    private:
        DrivetrainSubsystem* _drivetrain;
        Driver_Interface* _oi;
        bool _aiming;

        std::optional<frc::DriverStation::Alliance> alliance;

        wpi::array<frc::SwerveModulePosition, 4> _initial_positions = {
            frc::SwerveModulePosition{0_m, 0_rad},
            frc::SwerveModulePosition{0_m, 0_rad},
            frc::SwerveModulePosition{0_m, 0_rad},
            frc::SwerveModulePosition{0_m, 0_rad}
        };
};

#endif