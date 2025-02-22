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
#include "subsystems/FunnelSubsystem.h"

#include "commands/teleop/TeleopDriveCommand.h"

#include "commands/teleop/StowArmCommand.h"
#include "commands/teleop/TeleopProcessorCommand.h"
#include "commands/teleop/TeleopScoreCoralCommand.h"
#include "commands/teleop/TeleopIntakeCoralCommand.h"
#include "commands/teleop/TeleopIntakeAlgaeCommand.h"

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

        void OperatorPeriodic();
        void StartDriveState();
        void CancelDriverStates();
        void CancelOperatorStates();
        void StartOperatorState();

    private:
        // Subsystems
        #ifdef DRIVETRAIN_ENABLED   
        DrivetrainSubsystem _drivetrain{SwerveConstants::DrivetrainConstants::SWERVE_CONFIGS_ARRAY, _vision_ptr, SwerveConstants::DrivetrainConstants::PIGEON_ID, SwerveConstants::DrivetrainConstants::DRIVETRAIN_CANBUS_NAME};
        AutonGenerator _auton_generator{&_drivetrain};
        #endif

        #ifdef ELEVATOR_ENABLED
        ElevatorSubsystem _elevator{ElevatorConstants::PRIMARY_MOTOR_CAN_ID, ElevatorConstants::SECONDARY_MOTOR_CAN_ID, ElevatorConstants::HOME_SENSOR_DI_CH, ElevatorConstants::BRAKE_SERVO, ElevatorConstants::PID_C, ElevatorConstants::MAX_VELOCITY, ElevatorConstants::MAX_ACCELERATION, ElevatorConstants::FEED_FORWARD};
        #endif

        #ifdef INTAKE_ENABLED
        IntakeSubsystem _intake{IntakeConstants::MOTOR_CAN_ID, IntakeConstants::ALGAE_TOP_SENSOR_DI_CH, IntakeConstants::ALGAE_BOTTOM_SENSOR_DI_CH, IntakeConstants::CORAL_HIGH_SENSOR_DI_CH, IntakeConstants::CORAL_LOW_SENSOR_DI_CH};
        #endif

        #ifdef PIVOT_ENABLED
        PivotSubsystem _pivot{PivotConstants::PIVOT_MOTOR_CAN_ID, PivotConstants::PIVOT_HOME_DI_CH, PivotConstants::PID_C, PivotConstants::MAX_VELOCITY, PivotConstants::MAX_ACCELERATION, PivotConstants::FEED_FORWARD};
        #endif

        #ifdef FUNNEL_ENABLED
        FunnelSubsystem _funnel{FunnelConstants::MOTOR_CAN_ID, FunnelConstants::CORAL_SENSOR_DI_CH};
        #endif

        #ifdef VISION_ENABLED
        SC_Photon* _vision_ptr = new SC_Photon(VisionConstants::CAMERA_NAME, VisionConstants::APRIL_TAG_LAYOUT, VisionConstants::POSE_STRATEGY, VisionConstants::CAMERA_POSITION);
        #else
        SC_Photon* _vision_ptr = nullptr;
        #endif

        

        // Operator Interfaces
        Driver_Interface _oi_driver{};
        Operator_Interface _oi_operator{};
        Testing_Interface _oi_testing{};

        // Command Groups
        frc2::CommandPtr _stow_state_commands = frc2::cmd::Parallel(
            #if defined (ELEVATOR_ENABLED) && defined (PIVOT_ENABLED)
            StowArmCommand{&_pivot, &_elevator}.ToPtr(),
            #endif
            frc2::cmd::None()
        );

        frc2::CommandPtr _drive_state_commands = frc2::cmd::Parallel(
            #ifdef DRIVETRAIN_ENABLED
            TeleopDriveCommand{&_drivetrain, &_oi_driver}.ToPtr(),
            #endif
            frc2::cmd::None()
        );

        frc2::CommandPtr _drive_to_reef = frc2::cmd::None();
        frc2::CommandPtr _drive_to_feeder_station = frc2::cmd::None();
        frc2::CommandPtr _drive_to_processor = frc2::cmd::None();

        frc2::CommandPtr _intake_algae_commands = frc2::cmd::Parallel(
            #if defined (DRIVETRAIN_ENABLED) && defined (ELEVATOR_ENABLED) && defined (INTAKE_ENABLED) && defined (PIVOT_ENABLED)
            TeleopIntakeAlgaeCommand{&_drivetrain, &_elevator, &_intake, &_pivot, &_oi_operator}.ToPtr(),
            #endif
            frc2::cmd::None()
        );

        frc2::CommandPtr _intake_coral_commands = frc2::cmd::Parallel(
            #if defined (DRIVETRAIN_ENABLED) && defined (ELEVATOR_ENABLED) && defined (INTAKE_ENABLED) && defined (PIVOT_ENABLED) && defined (FUNNEL_ENABLED)
            TeleopIntakeCoralCommand{&_drivetrain, &_elevator, &_intake, &_pivot, &_funnel, &_oi_operator}.ToPtr(),
            #endif
            frc2::cmd::None()
        );

        frc2::CommandPtr _processor_commands = frc2::cmd::Parallel(
            #if defined (DRIVETRAIN_ENABLED) && defined (ELEVATOR_ENABLED) && defined (INTAKE_ENABLED) && defined (PIVOT_ENABLED)
            TeleopProcessorCommand{&_drivetrain, &_elevator, &_intake, &_pivot, &_oi_operator}.ToPtr(),
            #endif
            frc2::cmd::None()
        );

        frc2::CommandPtr _score_coral_commands = frc2::cmd::Parallel(
            #if defined (DRIVETRAIN_ENABLED) && defined (ELEVATOR_ENABLED )&& defined (INTAKE_ENABLED) &&  defined (PIVOT_ENABLED)
            TeleopScoreCoralCommand{&_drivetrain, &_elevator, &_intake, &_pivot, &_oi_operator}.ToPtr(),
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
        enum driver_states {
            drive,
            auto_pickup_coral, 
            auto_pickup_algae,
            auto_score_reef, 
            auto_score_processor
        }; //main state
        driver_states _driver_robot_state = drive;

        enum operator_states {
            stow,
            manual_score_coral,
            manual_score_processor, 
            manual_remove_algae,
            climb
        }; //state inside the drive state (driver)
        operator_states _operator_drive_robot_state = stow;

        // Power Stuff
        frc::PowerDistribution _pdp{1, frc::PowerDistribution::ModuleType::kRev};

        // Variables
        std::optional<frc2::CommandPtr> _auton_command;
};

#endif