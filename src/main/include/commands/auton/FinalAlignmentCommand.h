#ifndef FINALALIGNMENTCOMMAND_H
#define FINALALIGNMENTCOMMAND_H

#include "OI.h"
#include "Constants.h"
#include "subsystems/DrivetrainSubsystem.h"

#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

class FinalAlignmentCommand
    :public frc2::CommandHelper<frc2::Command, FinalAlignmentCommand> {

    public:
        explicit FinalAlignmentCommand(DrivetrainSubsystem* drivetrain_subsystem, frc::Pose2d target_pose);

        void Initialize() override;
        void Execute() override;
        void End(bool inturrupted) override;
        bool IsFinished() override;

    private:
        DrivetrainSubsystem* _drivetrain_subsystem;
		pathplanner::PPHolonomicDriveController* _drive_controller = new pathplanner::PPHolonomicDriveController(
			pathplanner::PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
			pathplanner::PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
		);
		frc::Pose2d _target_pose;
		int _counter = 0;

};

#endif