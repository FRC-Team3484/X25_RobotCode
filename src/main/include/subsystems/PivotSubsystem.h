#ifndef PIVOT_SUBSYSTEM_H
#define PIVOT_SUBSYSTEM_H

#include "Constants.h"
#include "FRC3484_Lib/utils/SC_Datatypes.h"

#include <frc/DigitalInput.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/Timer.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/controller/PIDController.h>
#include <frc/RobotController.h>

#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/TalonFX.hpp>

#include <units/angle.h>

/**
 * The pivot subsystem controls the pivot motion of the intake, allowing it to turn up and down, and prints test mode data 
 */

class PivotSubsystem : public frc2::SubsystemBase {
    public:
        /**
         * Creates an instance of the pivot subsystem
         * 
         * @param pivot_motor_can_id The CAN ID for the only pivot motor
         * @param pivot_home_di_ch The ID for the home sensor
         * @param pivot_pidc The pivot PID constants
         * @param max_velocity The maximum velocity the pivot can move
         * @param max_acceleration The maxium acceleration the pivot can move
         * @param feed_forward_constants The pivot feed forward constants
         */
        PivotSubsystem(
            int pivot_motor_can_id,
            int pivot_home_di_ch,
            SC::SC_PIDConstants pivot_pidc,
            units::radians_per_second_t max_velocity,
            units::radians_per_second_squared_t max_acceleration,
            SC::SC_AngularFeedForward feed_forward_constants
        );

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


        //test functions


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

        frc2::CommandPtr PsuedoSetAngle(std::function<double()> angle);

        frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction);
        frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction);

    private:

        bool _HomeSensor();
        bool _GetStalled();
        double _GetStallPercentage();
        
        units::degree_t _GetPivotAngle();
        units::degrees_per_second_t _GetPivotVelocity();

        ctre::phoenix6::hardware::TalonFX _pivot_motor;

        enum State {home, ready, test};
        State _pivot_state = home;

        frc::DigitalInput _pivot_home;
        
        frc::PIDController _pivot_pid_controller{0,0,0};

        frc::TrapezoidProfile<units::degree> _pivot_trapezoid;

        frc::TrapezoidProfile<units::degree>::State _intitial_state{Pivot::HOME_POSITION, 0_deg_per_s};
        frc::TrapezoidProfile<units::degree>::State _target_state{Pivot::TARGET_POSITION, 0_deg_per_s};
        frc::Timer _trapezoid_timer;

        frc::ArmFeedforward _pivot_feed_forward;

    frc2::sysid::SysIdRoutine m_sysIdRoutine{
        frc2::sysid::Config{std::nullopt, std::nullopt, std::nullopt, nullptr},
        frc2::sysid::Mechanism{
            [this](units::volt_t driveVoltage) {
            _pivot_motor.SetVoltage(driveVoltage);
            
        },
        [this](frc::sysid::SysIdRoutineLog* log) {
            log->Motor("primary-motor")
                .voltage(_pivot_motor.Get() * frc::RobotController::GetBatteryVoltage())
                .position(units::turn_t{_GetPivotAngle()})
                .velocity(units::turns_per_second_t{_GetPivotVelocity()});
        },
        this}};
};

#endif