#ifndef PIVOT_SUBSYSTEM_H
#define PIVOT_SUBSYSTEM_H

#include "Constants.h"
#include "FRC3484_Lib/utils/SC_Datatypes.h"
#include "FRC3484_Lib/components/SC_ArmFeedForward.h"

#include <frc/DigitalInput.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/Timer.h>
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
        void _SetPivotAngle(units::degree_t angle);

        bool _isHomed = false;
        
        units::degree_t _GetPivotAngle();
        units::degrees_per_second_t _GetPivotVelocity();

        ctre::phoenix6::hardware::TalonFX _pivot_motor;

        enum state {
            home, 
            ready, 
            test
        };
        state _pivot_state = home;

        units::degree_t _offset = 0_deg;

        frc::DigitalInput _pivot_home;
        
        frc::PIDController _pivot_pid_controller{0,0,0};

        frc::TrapezoidProfile<units::degree> _pivot_trapezoid;

        frc::TrapezoidProfile<units::degree>::State _intitial_state{PivotConstants::HOME_POSITION, 0_deg_per_s};
        frc::TrapezoidProfile<units::degree>::State _target_state{PivotConstants::HOME_POSITION /*this gets changes based apon user input*/, 0_deg_per_s};
        frc::Timer _trapezoid_timer;

        SC_ArmFeedForward _pivot_feed_forward;

        units::radians_per_second_t _previous_pivot_velocity = 0_rad_per_s;
};

#endif