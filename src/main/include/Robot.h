#ifndef ROBOT_H
#define ROBOT_H

#include "OI.h"
#include "Constants.h"
#include "Config.h"

#include <frc/TimedRobot.h>
#include <frc2/command/Commands.h>
#include <frc2/command/CommandPtr.h>
#include <frc/PowerDistribution.h>

#include "FRC3484_Lib/components/SC_Photon.h"

#include "subsystems/DrivetrainSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"
#include "subsystems/LEDs/LEDSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/PivotSubsystem.h"
#include "subsystems/FunnelSubsystem.h"

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
        #ifdef VISION_ENABLED
        SC_Photon* _vision_ptr = new SC_Photon(VisionConstants::CAMERA_CONFIGS, VisionConstants::APRIL_TAG_LAYOUT, VisionConstants::POSE_STRATEGY);
        #else
        SC_Photon* _vision_ptr = nullptr;
        #endif

        // Subsystems
        #ifdef DRIVETRAIN_ENABLED   
        DrivetrainSubsystem* _drivetrain = new DrivetrainSubsystem();
        #else
        DrivetrainSubsystem* _drivetrain = nullptr;
        AutonGenerator* _auton_generator = nullptr;
        #endif

        #ifdef ELEVATOR_ENABLED
        ElevatorSubsystem* _elevator= new ElevatorSubsystem(ElevatorConstants::PRIMARY_MOTOR_CAN_ID, ElevatorConstants::SECONDARY_MOTOR_CAN_ID, ElevatorConstants::HOME_SENSOR_DI_CH);
        #else
        ElevatorSubsystem* _elevator = nullptr;
        #endif

        #ifdef INTAKE_ENABLED
        IntakeSubsystem* _intake = new IntakeSubsystem(IntakeConstants::MOTOR_CAN_ID, IntakeConstants::ALGAE_TOP_SENSOR_DI_CH, IntakeConstants::ALGAE_BOTTOM_SENSOR_DI_CH, IntakeConstants::CORAL_HIGH_SENSOR_DI_CH, IntakeConstants::CORAL_LOW_SENSOR_DI_CH);
        #else
        IntakeSubsystem* _intake = nullptr;
        #endif

        #ifdef PIVOT_ENABLED
        PivotSubsystem* _pivot = new PivotSubsystem(PivotConstants::PIVOT_MOTOR_CAN_ID, PivotConstants::PIVOT_HOME_DI_CH);
        #else
        PivotSubsystem* _pivot = nullptr;
        #endif

        #ifdef FUNNEL_ENABLED
        FunnelSubsystem* _funnel = new FunnelSubsystem(FunnelConstants::MOTOR_CAN_ID, FunnelConstants::CORAL_SENSOR_DI_CH);
        #else
        FunnelSubsystem* _funnel = nullptr;
        #endif

        LEDSubsystem _leds{LEDConstants::LED_PWM_PORT, LEDConstants::LED_STRIP_LENGTH};

        // Operator Interfaces
        Driver_Interface* _oi_driver = new Driver_Interface();
        Operator_Interface* _oi_operator = new Operator_Interface();
        Testing_Interface* _oi_testing = new Testing_Interface();

        // Power Stuff
        frc::PowerDistribution _pdp{1, frc::PowerDistribution::ModuleType::kRev};

        // Variables
        std::optional<frc2::CommandPtr> _auton_command;
};

#endif