#ifndef ELEVATOR_SUBSYSTEM_H
#define ELEVATOR_SUBSYSTEM_H

#include <frc2/command/SubsystemBase.h>
#include <frc/DigitalInput.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/Timer.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <units/length.h>
#include <frc/controller/PIDController.h>
#include <frc/Servo.h>
#include <units/velocity.h>

#include <ctre/phoenix6/TalonFX.hpp>

#include "Constants.h"
#include "FRC3484_Lib/utils/SC_Datatypes.h"

class ElevatorSubsystem : public frc2::SubsystemBase {
    public:
        /**
         * Creates an instance of the elevator subsystem which controls the elevator, allowing it to move up and down
         * 
         * @param primary_motor_can_id The CAN ID for the primary motor
         * @param secondary_motor_can_id The CAN ID for the secondary motor
         * @param home_sensor_di_ch The ID for the home sensor
         * @param brake_servo The ID for the brake servo
         * @param elevator_pidc The elevator PID constants
         * @param max_velocity The maximum velocity the elevator can move
         * @param max_acceleration The maxium acceleration the elevator can move
         * @param feed_forward_constants The elevator feed forward constants
         */
        ElevatorSubsystem(
            int primary_motor_can_id,
            int secondary_motor_can_id,
            int home_sensor_di_ch,
            int brake_servo,
            SC::SC_PIDConstants elevator_pidc,
            units::feet_per_second_t max_velocity,
            units::feet_per_second_squared_t max_acceleration,
            SC::SC_LinearFeedForward feed_forward_constants
        );

        /**
         * Sets the height of the elevator
         * 
         * @param height The height to set the elevator, in inches
         */
        void SetHeight(units::inch_t height);
        /**
         * Checks if the elevator is at the target height
         * 
         * @return True if the elevator has reached the target
         */
        bool AtTargetHeight();

        /**
         * Checks if the elevator is able to stow the pivot
         * 
         * @return True if the elevator has reached the target
         */
        bool AtSafeStowPosition();

        /**
         * Sets the power of the elevator
         * 
         * @param power The power of the elevator, as a double
         */
        void SetPower(double power);
        
        /**
         * Sets the test mode of the elevator subsystem
         * 
         * @param test_mode If test mode should be enabled or not
         */
        void SetTestMode(bool test_mode);

        /**
         * Prints the test info to Smart Dashboard, used when the robot is in test mode
         */
        void PrintTestInfo();

        /**
         * Sets the elevator state to home
         */
        void SetElevatorToHome();

        void Periodic() override;

    private:
        bool _climbing = false;
        units::inch_t _offset = 0_in;
        bool _HomeSensor();
        bool _GetStalled();
        double _GetStallPercentage();
        void _SetPosition(units::inch_t offset);

        bool _isHomed = false;
        
        units::inch_t _GetElevatorHeight();
        units::feet_per_second_t _GetElevatorVelocity();

        ctre::phoenix6::hardware::TalonFX _primary_motor;
        ctre::phoenix6::hardware::TalonFX _secondary_motor;

        enum state {
            home, 
            ready, 
            test
        };
        state _elevator_state = home;

        frc::DigitalInput _home_sensor;

        frc::Servo _brake_servo;

        frc::PIDController _elevator_pid_controller{0,0,0};

        frc::TrapezoidProfile<units::feet> _elevator_trapezoid;
        frc::TrapezoidProfile<units::feet>::State _initial_state {ElevatorConstants::HOME_POSITION, 0_fps};
        frc::TrapezoidProfile<units::feet>::State _target_state {ElevatorConstants::HOME_POSITION, 0_fps};
        frc::Timer _trapezoid_timer;

        frc::ElevatorFeedforward _elevator_feed_forward;

        units::meters_per_second_t _previous_elevator_velocity = 0_mps;

};

#endif