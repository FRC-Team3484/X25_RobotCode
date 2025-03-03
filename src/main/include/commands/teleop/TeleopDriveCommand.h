#ifndef DRIVE_COMMAND_H
#define DRIVE_COMMAND_H

#include "OI.h"
#include "subsystems/DrivetrainSubsystem.h"

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/DriverStation.h>
#include <frc/geometry/Translation2d.h>

class TeleopDriveCommand: public frc2::CommandHelper<frc2::Command, TeleopDriveCommand> {
    public:
        /**
         * Controls the drivetrain in teleop
         * Turns the button inputs from the driver controller into actions to drive the robot around
         * 
         * @param drivetrain A pointer to the drivetrain subsystem
         * @param oi A pointer to the driver interface
         */
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

        frc::Translation2d _pivot_corner{0_m, 0_m};
        frc::Translation2d _pivot_drive{0_m, 0_m};

        enum State {drive, pivot};
        State _drivetrain_state = drive;

        wpi::array<frc::SwerveModulePosition, 4> _initial_positions = {
            frc::SwerveModulePosition{0_m, 0_rad},
            frc::SwerveModulePosition{0_m, 0_rad},
            frc::SwerveModulePosition{0_m, 0_rad},
            frc::SwerveModulePosition{0_m, 0_rad}
        };
};

#endif