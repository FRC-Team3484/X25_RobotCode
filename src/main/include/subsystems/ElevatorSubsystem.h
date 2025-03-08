#ifndef ELEVATOR_SUBSYSTEM_H
#define ELEVATOR_SUBSYSTEM_H

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc/DigitalInput.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/Timer.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <units/length.h>
#include <frc/controller/PIDController.h>
#include <frc/Servo.h>
#include <frc/RobotController.h>

#include <frc2/command/Commands.h>

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
         */
        ElevatorSubsystem(
            int primary_motor_can_id,
            int secondary_motor_can_id,
            int home_sensor_di_ch
        );

        frc2::CommandPtr PseudoMoveCommand(std::function<double()> power);
        frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction);
        frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction);
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

        void SetVoltage(units::volt_t voltage);

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

        void Periodic() override;

    private:
        bool _climbing = false;
        units::inch_t _offset = 0_in;
        bool _HomeSensor();
        bool _GetStalled();
        double _GetStallPercentage();
        void _SetPosition(units::inch_t offset);

        units::inch_t _GetElevatorHeight();
        units::feet_per_second_t _GetElevatorVelocity();

        ctre::phoenix6::hardware::TalonFX _primary_motor;
        ctre::phoenix6::hardware::TalonFX _secondary_motor;

        frc::DigitalInput _home_sensor;

        frc2::sysid::SysIdRoutine _sysid_routine{
            frc2::sysid::Config{0.2_V/1_s, 3_V, std::nullopt, nullptr}, 
            frc2::sysid::Mechanism{
                [this](units::volt_t voltage){  
                _primary_motor.SetVoltage(voltage);
                },

                [this](frc::sysid::SysIdRoutineLog* log){
                    log->Motor("primary_motor")
                    .voltage(_primary_motor.Get() * frc::RobotController::GetBatteryVoltage())
                    .position(units::meter_t{_GetElevatorHeight()})
                    .velocity(units::meters_per_second_t{_GetElevatorVelocity()});
                },
            this}
        };

};

#endif