#ifndef ROBOT_H
#define ROBOT_H

#include "OI.h"
#include "Constants.h"
#include "Config.h"
#include "AutonGenerator.h"

#include <frc/TimedRobot.h>
#include <frc2/command/Commands.h>
#include <frc2/command/CommandPtr.h>
#include <frc/PowerDistribution.h>

#include "FRC3484_Lib/components/SC_Photon.h"

#include "subsystems/LEDs/LEDSubsystem.h"

#include "subsystems/DrivetrainSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/PivotSubsystem.h"
#include "subsystems/FunnelSubsystem.h"

#include "commands/teleop/TeleopDriveCommand.h"

#include "commands/teleop/ClimbUpCommand.h"
#include "commands/teleop/StowArmCommand.h"
#include "commands/teleop/TeleopProcessorCommand.h"
#include "commands/teleop/TeleopScoreCoralCommand.h"
#include "commands/teleop/TeleopIntakeCoralCommand.h"
#include "commands/teleop/TeleopIntakeAlgaeCommand.h"

#include "commands/auton/AutonBasicScoreCoralCommand.h"

#include "commands/testing/TestElevatorCommand.h"
#include "commands/testing/TestIntakeCommand.h"
#include "commands/testing/TestPivotCommand.h"

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

        void OperatorPeriodic();
        void CancelOperatorCommands();
        void StartOperatorState();
        void StartDriveState();
        void CancelDriverCommands();

        void LedPeriodic();

        bool AutoGetLoadCoralCondition();
        bool AutoGetRemoveAlgaeCondition();
        bool AutoGetScoreReefCondition();
        //bool AutoGetScoreProcessorCondition();

        bool ManualGetLoadCoralCondition();
        bool ManualGetRemoveAlgaeCondition();
        bool ManualGetScoreReefCondition();
        //bool ManualGetScoreProcessorCondition();
        bool ManualGetClimbUpCondition();

        void ResetAllSubsystems();

    private:
        // Operator Interfaces
        Driver_Interface* _oi_driver = new Driver_Interface();
        Operator_Interface* _oi_operator = new Operator_Interface();
        Testing_Interface* _oi_testing = new Testing_Interface();

        #ifdef VISION_ENABLED
        SC_Photon* _vision_ptr = new SC_Photon(VisionConstants::CAMERA_CONFIGS, VisionConstants::APRIL_TAG_LAYOUT, VisionConstants::POSE_STRATEGY);
        #else
        SC_Photon* _vision_ptr = nullptr;
        #endif

        // Subsystems
        #ifdef DRIVETRAIN_ENABLED   
        DrivetrainSubsystem* _drivetrain = new DrivetrainSubsystem(SwerveConstants::DrivetrainConstants::SWERVE_CONFIGS_ARRAY, _vision_ptr, SwerveConstants::DrivetrainConstants::PIGEON_ID, SwerveConstants::DrivetrainConstants::DRIVETRAIN_CANBUS_NAME, _oi_operator);
        #else
        DrivetrainSubsystem* _drivetrain = nullptr;
        #endif

        #if defined (ELEVATOR_ENABLED) && defined (PIVOT_ENABLED)
        ElevatorSubsystem* _elevator = new ElevatorSubsystem(ElevatorConstants::PRIMARY_MOTOR_CAN_ID, ElevatorConstants::SECONDARY_MOTOR_CAN_ID, ElevatorConstants::HOME_SENSOR_DI_CH, ElevatorConstants::BRAKE_SERVO, ElevatorConstants::PID_C, ElevatorConstants::MAX_VELOCITY, ElevatorConstants::MAX_ACCELERATION, ElevatorConstants::FEED_FORWARD);
        PivotSubsystem* _pivot = new PivotSubsystem(PivotConstants::PIVOT_MOTOR_CAN_ID, PivotConstants::PIVOT_HOME_DI_CH, PivotConstants::PID_C, PivotConstants::MAX_VELOCITY, PivotConstants::MAX_ACCELERATION, PivotConstants::FEED_FORWARD, _elevator);
        #else
        ElevatorSubsystem* _elevator = nullptr;
        PivotSubsystem* _pivot = nullptr;
        #endif

        #ifdef INTAKE_ENABLED
        IntakeSubsystem* _intake = new IntakeSubsystem(IntakeConstants::MOTOR_CAN_ID, IntakeConstants::ALGAE_TOP_SENSOR_DI_CH, IntakeConstants::ALGAE_BOTTOM_SENSOR_DI_CH, IntakeConstants::CORAL_HIGH_SENSOR_DI_CH, IntakeConstants::CORAL_LOW_SENSOR_DI_CH);
        #else
        IntakeSubsystem* _intake = nullptr;
        #endif

        #ifdef FUNNEL_ENABLED
        FunnelSubsystem* _funnel = new FunnelSubsystem(FunnelConstants::MOTOR_CAN_ID, FunnelConstants::CORAL_SENSOR_DI_CH);
        #else
        FunnelSubsystem* _funnel = nullptr;
        #endif

        #ifdef LED_ENABLED
        LEDSubsystem* _leds = new LEDSubsystem(LEDConstants::LED_PWM_PORT, LEDConstants::LED_STRIP_LENGTH);
        #else
        LEDSubsystem* _leds = nullptr;
        #endif

        #if defined (DRIVETRAIN_ENABLED) && defined (INTAKE_ENABLED) && defined (PIVOT_ENABLED) && defined (ELEVATOR_ENABLED)
        AutonGenerator* _auton_generator = new AutonGenerator(_drivetrain, _elevator, _pivot, _intake);
        #else
        AutonGenerator* _auton_generator = nullptr;
        #endif
        
        // Command Groups
        frc2::CommandPtr _climb_up_state_commands = frc2::cmd::Parallel(
            #ifdef COMMANDS_ENABLED
            #ifdef ELEVATOR_ENABLED
            ClimbUpCommand{_elevator, _pivot}.ToPtr(),
            #endif
            #endif
            frc2::cmd::None()
        );
        
        frc2::CommandPtr _stow_state_commands = frc2::cmd::Parallel(
            #ifdef COMMANDS_ENABLED
            #if defined (ELEVATOR_ENABLED) && defined (PIVOT_ENABLED)
            StowArmCommand{_pivot, _elevator}.ToPtr(),
            #endif
            #endif
            frc2::cmd::None()
        );

        frc2::CommandPtr _drive_state_commands = frc2::cmd::Parallel(
            #ifdef DRIVETRAIN_ENABLED
            TeleopDriveCommand{_drivetrain, _oi_driver}.ToPtr(),
            #endif
            frc2::cmd::None()
        );

        frc2::CommandPtr _intake_algae_commands = frc2::cmd::Parallel(
            #ifdef COMMANDS_ENABLED
            #if defined (ELEVATOR_ENABLED) && defined (INTAKE_ENABLED) && defined (PIVOT_ENABLED)
            TeleopIntakeAlgaeCommand{_elevator, _intake, _pivot, _oi_operator}.ToPtr(),
            #endif
            #endif
            frc2::cmd::None()
        );

        frc2::CommandPtr _intake_coral_commands = frc2::cmd::Parallel(
            #ifdef COMMANDS_ENABLED
            #if  defined (ELEVATOR_ENABLED) && defined (INTAKE_ENABLED) && defined (PIVOT_ENABLED)
            TeleopIntakeCoralCommand{_elevator, _intake, _pivot, _funnel, _oi_operator}.ToPtr(),
            #endif
            #endif
            frc2::cmd::None()
        );

        frc2::CommandPtr _processor_commands = frc2::cmd::Parallel(
            #ifdef COMMANDS_ENABLED
            #if  defined (ELEVATOR_ENABLED) && defined (INTAKE_ENABLED) && defined (PIVOT_ENABLED)
            TeleopProcessorCommand{_elevator, _intake, _pivot, _oi_operator}.ToPtr(),
            #endif
            #endif
            frc2::cmd::None()
        );

        frc2::CommandPtr _score_coral_commands = frc2::cmd::Parallel(
            #ifdef COMMANDS_ENABLED
            #if  defined (ELEVATOR_ENABLED )&& defined (INTAKE_ENABLED) &&  defined (PIVOT_ENABLED)
            TeleopScoreCoralCommand{_elevator, _intake, _pivot, _oi_operator}.ToPtr(),
            #endif
            #endif
            frc2::cmd::None()
        );

        frc2::CommandPtr _test_state_commands = frc2::cmd::Parallel(
            #ifdef ELEVATOR_ENABLED
            TestElevatorCommand{_elevator, _oi_testing}.ToPtr(),
            #endif
            #ifdef INTAKE_ENABLED
            TestIntakeCommand{_intake, _oi_testing}.ToPtr(),
            #endif
            #ifdef PIVOT_ENABLED
            TestPivotCommand{_pivot, _oi_testing}.ToPtr(),
            #endif
            #ifdef DRIVETRAIN_ENABLED
            TeleopDriveCommand{_drivetrain, _oi_driver}.ToPtr(),
            #endif
            frc2::cmd::None()
        );

        // State machine
        enum driver_states {
            drive,
            auto_pickup_coral_driver, 
            auto_reef_driver,
            auto_score_processor_driver
        }; //main state
        driver_states _driver_robot_state = drive;

        enum operator_states {
            stow,
            manual_score_coral,
            manual_score_processor, 
            manual_remove_algae,
            manual_pickup_coral,
            climb_up,
            climb_down,
            auto_pickup_coral_operator, 
            auto_pickup_algae_operator,
            auto_score_reef_operator, 
            auto_score_processor_operator
        }; //state inside the drive state (driver)
        operator_states _operator_drive_robot_state = stow;

        // Power Stuff
        frc::PowerDistribution _pdp{1, frc::PowerDistribution::ModuleType::kRev};
        bool _low_battery = false;

        bool _has_been_enabled = false;

        // Variables
        std::optional<frc2::CommandPtr> _auton_command;

        frc::SendableChooser<frc::Pose2d> _auton_start_positions;
        frc::SendableChooser<std::string_view> _basic_autons;

        // Elastic Dashboard Stuff
        units::second_t _match_time = 0_s;
};

#endif