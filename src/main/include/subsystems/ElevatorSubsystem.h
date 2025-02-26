// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#ifndef ELEVATORSUBSYSTEM_H
#define ELEVATORSUBSYSTEM_H

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc/DigitalInput.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/Timer.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <units/length.h>
#include <frc/controller/PIDController.h>
#include <frc/RobotController.h>

#include <ctre/phoenix6/TalonFX.hpp>

#include "Constants.h"
#include "FRC3484_Lib/utils/SC_Datatypes.h"
#include "OI.h"

class ElevatorSubsystem : public frc2::SubsystemBase {
    public:
        ElevatorSubsystem(
            int primary_motor_can_id,
            int secondary_motor_can_id,
            int home_sensor_di_ch,
            SC::SC_PIDConstants elevator_pidc,
            units::feet_per_second_t max_velocity,
            units::feet_per_second_squared_t max_acceleration,
            SC::SC_LinearFeedForward feed_forward_constants
            );
        void SetHeight(units::inch_t height);
        bool AtTargetHeight();
        void SetPower(double power);
        // sets power of motor for elevator
        void SetTestMode(bool test_mode);
        void Periodic() override;
        void PrintTestInfo();



        frc2::CommandPtr PsuedoSetPower(std::function<double()> height);

        frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction);
        frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction);

    private:
        
        bool _HomeSensor();
        bool _GetStalled();
        double _GetStallPercentage();

        units::inch_t _GetElevatorHeight();
        units::feet_per_second_t _GetElevatorVelocity();

        ctre::phoenix6::hardware::TalonFX _primary_motor;
        ctre::phoenix6::hardware::TalonFX _secondary_motor;

        enum State {home, ready, test};
        State _elevator_state = home;

        frc::DigitalInput _home_sensor;

        frc::PIDController _elevator_pid_controller{0,0,0};

        frc::TrapezoidProfile<units::feet> _elevator_trapezoid;

        frc::TrapezoidProfile<units::feet>::State _initial_state {Elevator::HOME_POSITION, 0_fps};
        frc::TrapezoidProfile<units::feet>::State _target_state {Elevator::HOME_POSITION, 0_fps};
        frc::Timer _trapezoid_timer;
        Testing_Interface* _testing_interface;

        frc::ElevatorFeedforward _elevator_feed_forward;



    frc2::sysid::SysIdRoutine _sysIdRoutine{
        frc2::sysid::Config{std::nullopt, std::nullopt, std::nullopt, nullptr},
        frc2::sysid::Mechanism{
            [this](units::volt_t driveVoltage) {
            _primary_motor.SetVoltage(driveVoltage);
            _secondary_motor.SetVoltage(driveVoltage);
            
        },
        [this](frc::sysid::SysIdRoutineLog* log) {
            log->Motor("primary-motor")
                .voltage(_primary_motor.Get() * frc::RobotController::GetBatteryVoltage())
                .position(units::meter_t{_GetElevatorHeight()})
                .velocity(units::meters_per_second_t{_GetElevatorVelocity()});
            log->Motor("secondary-motor")
                .voltage(_secondary_motor.Get() * frc::RobotController::GetBatteryVoltage())
                .position(units::meter_t{_GetElevatorHeight()})
                .velocity(units::meters_per_second_t{_GetElevatorVelocity()});
        },
        this}};

};

#endif