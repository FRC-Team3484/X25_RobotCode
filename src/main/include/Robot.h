#ifndef ROBOT_H
#define ROBOT_H

#include "OI.h"
#include "Constants.h"
#include "Config.h"
#include "AutonGenerator.h"

#include <frc/TimedRobot.h>
#include <frc2/command/Commands.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc/PowerDistribution.h>

#include "FRC3484_Lib/components/SC_Photon.h"

#include "subsystems/DrivetrainSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/PivotSubsystem.h"

#include "commands/teleop/TeleopDriveCommand.h"

#include "commands/testing/TestElevatorCommand.h"
#include "commands/testing/TestIntakeCommand.h"
#include "commands/testing/TestPivotCommand.h"

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
        // Subsystems
        #ifdef ELEVATOR_ENABLED
        ElevatorSubsystem _elevator{ElevatorConstants::PRIMARY_MOTOR_CAN_ID, ElevatorConstants::SECONDARY_MOTOR_CAN_ID, ElevatorConstants::HOME_SENSOR_DI_CH, ElevatorConstants::PID_C, ElevatorConstants::MAX_VELOCITY, ElevatorConstants::MAX_ACCELERATION, ElevatorConstants::FEED_FORWARD};
        #endif

        #ifdef INTAKE_ENABLED
        IntakeSubsystem _intake{IntakeConstants::MOTOR_CAN_ID, IntakeConstants::ALGAE_TOP_SENSOR_DI_CH, IntakeConstants::ALGAE_BOTTOM_SENSOR_DI_CH, IntakeConstants::CORAL_HIGH_SENSOR_DI_CH, IntakeConstants::CORAL_LOW_SENSOR_DI_CH};
        #endif

        #ifdef PIVOT_ENABLED
        PivotSubsystem _pivot{PivotConstants::PIVOT_MOTOR_CAN_ID, PivotConstants::PIVOT_HOME_DI_CH, PivotConstants::PID_C, PivotConstants::MAX_VELOCITY, PivotConstants::MAX_ACCELERATION, PivotConstants::FEED_FORWARD};
        #endif

        #ifdef VISION_ENABLED
        SC_Photon* _vision_ptr = new SC_Photon(VisionConstants::CAMERA_NAME, VisionConstants::APRIL_TAG_LAYOUT, VisionConstants::POSE_STRATEGY, VisionConstants::CAMERA_POSITION);
        #else
        SC_Photon* _vision_ptr = nullptr;
        #endif

        #ifdef DRIVETRAIN_ENABLED   
        DrivetrainSubsystem _drivetrain{SwerveConstants::DrivetrainConstants::SWERVE_CONFIGS_ARRAY, _vision_ptr};
        AutonGenerator _auton_generator{&_drivetrain};
        #endif

        // Operator Interfaces
        Driver_Interface _oi_driver{};
        Operator_Interface _oi_operator{};
        Testing_Interface _oi_testing{};

        // Command Groups
        frc2::CommandPtr _drive_state_commands = frc2::cmd::Parallel(
            #ifdef DRIVETRAIN_ENABLED
            TeleopDriveCommand{&_drivetrain, &_oi_driver}.ToPtr(),
            #endif
            frc2::cmd::None()
        );

        frc2::CommandPtr _test_state_commands = frc2::cmd::Parallel(
            #ifdef ELEVATOR_ENABLED
            TestElevatorCommand{&_elevator, &_oi_testing}.ToPtr(),
            #endif
            #ifdef INTAKE_ENABLED
            TestIntakeCommand{&_intake, &_oi_testing}.ToPtr(),
            #endif
            #ifdef PIVOT_ENABLED
            TestPivotCommand{&_pivot, &_oi_testing}.ToPtr(),
            #endif
            frc2::cmd::None()
        );

        // State machine
        enum State {drive};
        State _robot_state = drive;

        // Power Stuff
        frc::PowerDistribution _pdp{1, frc::PowerDistribution::ModuleType::kRev};

        // Variables
        std::optional<frc2::CommandPtr> _auton_command;
};

#endif