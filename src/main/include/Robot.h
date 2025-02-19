// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#ifndef ROBOT_H
#define ROBOT_H

#include "OI.h"
#include "Constants.h"
#include "Config.h"
#include "AutonGenerator.h"

#include "subsystems/DrivetrainSubsystem.h"
#include "commands/teleop/TeleopDriveCommand.h"
#include "FRC3484_Lib/components/SC_Photon.h"

#include <frc/TimedRobot.h>
#include "frc2/command/Commands.h"
#include <frc2/command/CommandPtr.h>
#include <frc/PowerDistribution.h>

#include <units/time.h>

class Robot : public frc::TimedRobot {
    public:
        Robot();
        void RobotPeriodic() override;
        void DisabledInit() override;
        void DisabledPeriodic() override;
        void DisabledExit() override;
        void AutonomousInit() override;
        void AutonomousPeriodic() override;
        void AutonomousExit() override;
        void TeleopInit() override;
        void TeleopPeriodic() override;
        void TeleopExit() override;
        void TestInit() override;
        void TestPeriodic() override;
        void TestExit() override;

    private:
        // State machine
        enum State {drive};
        State _robot_state = drive;

        // Power Stuff
        frc::PowerDistribution _pdp{1, frc::PowerDistribution::ModuleType::kRev};

        // Interface OI
        Driver_Interface _oi_driver{};
        Operator_Interface _oi_operator{};

        // Vision
        #ifdef VISION_ENABLED
        SC_Photon* _vision_ptr = new SC_Photon(VisionConstants::CAMERA_NAME, VisionConstants::APRIL_TAG_LAYOUT, VisionConstants::POSE_STRATEGY, VisionConstants::CAMERA_POSITION);
        #else
        SC_Photon* _vision_ptr = nullptr;
        #endif

        // Subsystems
        #ifdef DRIVETRAIN_ENABLED   
        DrivetrainSubsystem _drivetrain{SwerveConstants::DrivetrainConstants::SWERVE_CONFIGS_ARRAY, _vision_ptr};
        AutonGenerator _auton_generator{&_drivetrain};
        #endif

        

        // Command Groups
        frc2::CommandPtr _drive_state_commands = frc2::cmd::Parallel(
            #ifdef DRIVETRAIN_ENABLED
            TeleopDriveCommand{&_drivetrain, &_oi_driver}.ToPtr(),
            #endif
            frc2::cmd::None()
        );

        // Variables
        std::optional<frc2::CommandPtr> _auton_command;

        // Elastic Dashboard Stuff
        units::second_t _match_time = 0_s;
};

#endif