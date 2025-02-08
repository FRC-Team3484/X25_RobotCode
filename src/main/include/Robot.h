#ifndef ROBOT_H
#define ROBOT_H

#include <optional>
#include "Constants.h"
#include "Config.h"

#include "subsystems/IntakeSubsystem.h"
#include "OI.h"
#include "commands/testing/TestIntakeCommand.h"

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>

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
        #ifdef INTAKE_ENABLED
        IntakeSubsystem _intake{IntakeConstants::MOTOR_CAN_ID, IntakeConstants::ALGAE_TOP_SENSOR_DI_CH, IntakeConstants::ALGAE_BOTTOM_SENSOR_DI_CH, IntakeConstants::CORAL_HIGH_SENSOR_DI_CH, IntakeConstants::CORAL_LOW_SENSOR_DI_CH};
        #endif

        // Operator Interfaces
        Testing_Interface _oi_testing{};

        // Command Groups
        frc2::CommandPtr _test_state_commands = frc2::cmd::Parallel(
            #ifdef INTAKE_ENABLED
            TestIntakeCommand{&_intake, &_oi_testing}.ToPtr(),
            #endif
            frc2::cmd::None()
        );        
};

#endif