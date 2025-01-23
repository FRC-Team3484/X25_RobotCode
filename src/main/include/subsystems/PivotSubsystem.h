#ifndef PIVOT_SUBSYSTEM_H
#define PIVOT_SUBSYSTEM_H

#include "Constants.h"
#include "FRC3484_Lib/utils/SC_Datatypes.h"

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
        PivotSubsystem(
            int pivot_motor_can_id,
            int pivot_home_di_ch,
            SC::SC_PIDConstants pivot_pidc,
            units::radians_per_second_t max_velocity,
            units::radians_per_second_squared_t max_acceleration,
            SC::SC_AngularFeedForward feed_forward_constants
        );

        

        void SetPivotAngle(units::degree_t angle);
        bool AtTargetPosition();
        void Periodic() override;
        //test functions
        void SetPower(double power);
        void SetTestMode(bool test_mode);
        void PrintTestInfo();

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
};

#endif