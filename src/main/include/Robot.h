#ifndef ROBOT_H
#define ROBOT_H

#include <optional>

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>

#include "subsystems/PivotSubsystem.h"
#include "commands/testing/TestPivotCommand.h"
#include "Config.h"
#include "Constants.h"
#include "OI.h"

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

        #ifdef PIVOT
            PivotSubsystem _pivot{Pivot::PIVOT_MOTOR_CAN_ID, Pivot::PIVOT_HOME_DI_CH, Pivot::PID_C, Pivot::MAX_VELOCITY, Pivot::MAX_ACCELERATION, Pivot::FEED_FORWARD};
        #endif

        // Operator Interfaces
        Testing_Interface _oi_testing{};
        
        
        // Command Groups
        frc2::CommandPtr _test_state_commands = frc2::cmd::Parallel(
            #ifdef PIVOT
            TestPivotCommand{&_pivot, &_oi_testing}.ToPtr(),
            #endif
            frc2::cmd::None()
        );   
};

#endif