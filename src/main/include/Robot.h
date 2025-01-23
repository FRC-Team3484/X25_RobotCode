#ifndef ROBOT_H
#define ROBOT_H

#include <optional>
#include "Constants.h"
#include "Config.h"

#include "subsystems/ElevatorSubsystem.h"
#include "OI.h"
#include "commands/testing/TestElevatorCommand.h"

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
        #ifdef ELEVATOR_ENABLED
        ElevatorSubsystem _elevator{ElevatorConstants::PRIMARY_MOTOR_CAN_ID, ElevatorConstants::SECONDARY_MOTOR_CAN_ID, ElevatorConstants::HOME_SENSOR_DI_CH, ElevatorConstants::PID_C, ElevatorConstants::MAX_VELOCITY, ElevatorConstants::MAX_ACCELERATION, ElevatorConstants::FEED_FORWARD};
        #endif

        // Operator Interfaces
        Testing_Interface _oi_testing{};

        // Command Groups
        frc2::CommandPtr _test_state_commands = frc2::cmd::Parallel(
            #ifdef ELEVATOR_ENABLED
            TestElevatorCommand{&_elevator, &_oi_testing}.ToPtr(),
            #endif
            frc2::cmd::None()
        );   
};

#endif