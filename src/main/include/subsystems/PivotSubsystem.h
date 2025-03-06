#ifndef PIVOT_SUBSYSTEM_H
#define PIVOT_SUBSYSTEM_H

#include "Constants.h"
#include "FRC3484_Lib/utils/SC_Datatypes.h"


#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc2/command/Commands.h>
#include <frc/RobotController.h>

#include <frc/DigitalInput.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/Timer.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/controller/PIDController.h>

#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/TalonFX.hpp>

#include <units/angle.h>

class PivotSubsystem : public frc2::SubsystemBase {
    public:
        /**
         * Creates an instance of the pivot subsystem, which controls the up and down pivot motion to move the intake
         * 
         * @param pivot_motor_can_id The CAN ID for the only pivot motor
         * @param pivot_home_di_ch The ID for the home sensor
         */
        PivotSubsystem(
            int pivot_motor_can_id,
            int pivot_home_di_ch
        );

        frc2::CommandPtr PseudoMoveCommand(std::function<double()> power);
        frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction);
        frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction);

        /**
         * Sets the angle of the pivot
         * 
         * @param angle The angle to set the pivot, in degrees
         */
        void SetPivotAngle(units::degree_t angle);

        /**
         * Checks if the pivot is at the target position
         * 
         * @return True if the pivot has reached the position
         */
        bool AtTargetPosition();

        /**
         * Sets the power of the pivot
         * 
         * @param power The power of the pivot, as a double
         */
        void SetPower(double power);

        /**
         * Sets the test mode of the pivot subsystem
         * 
         * @param test_mode If test mode should be enabled or not
         */
        void SetTestMode(bool test_mode);

        /**
         * Prints the test info to Smart Dashboard, used when the robot is in test mode
         */
        void PrintTestInfo();

        void Periodic() override;

    private:
        bool _HomeSensor();
        bool _GetStalled();
        double _GetStallPercentage();
        
        units::degree_t _GetPivotAngle();
        units::degrees_per_second_t _GetPivotVelocity();

        ctre::phoenix6::hardware::TalonFX _pivot_motor;

        frc::DigitalInput _pivot_home;

        frc2::sysid::SysIdRoutine _sysid_routine{
            frc2::sysid::Config{std::nullopt, std::nullopt, std::nullopt, nullptr}, 
            frc2::sysid::Mechanism{
                [this](units::volt_t voltage){  
                _pivot_motor.SetVoltage(voltage);
                },

                [this](frc::sysid::SysIdRoutineLog* log){
                    log->Motor("pivot motor")
                    .voltage(_pivot_motor.Get() * frc::RobotController::GetBatteryVoltage())
                    .position(units::turn_t{_GetPivotAngle()})
                    .velocity(units::turns_per_second_t{_GetPivotVelocity()});
                },
            this}
        };
};

#endif