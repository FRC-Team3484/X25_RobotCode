// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ElevatorSubsystem.h"
#include <units/math.h>

using namespace ctre::phoenix6;
using namespace Elevator;
using namespace units;

ElevatorSubsystem::ElevatorSubsystem(
    int primary_motor_can_id,
    int secondary_motor_can_id,
    int home_sensor_di_ch,
    SC::SC_PIDConstants elevator_pidc,
    feet_per_second_t max_velocity,
    feet_per_second_squared_t max_acceleration,
    SC::SC_LinearFeedForward feed_forward_constants
    ) : 
    _primary_motor(primary_motor_can_id),
    _secondary_motor(secondary_motor_can_id),
    _home_sensor(home_sensor_di_ch),
    _elevator_pid_controller{elevator_pidc.Kp, elevator_pidc.Ki, elevator_pidc.Kd},
    _elevator_trapezoid{{max_velocity, max_acceleration}},
    _elevator_feed_forward{
        feed_forward_constants.S,
        feed_forward_constants.G,
        feed_forward_constants.V,
        feed_forward_constants.A
    }
    {
        configs::TalonFXConfiguration motor_config{};
        motor_config.MotorOutput.Inverted = INVERT_MOTORS;
        motor_config.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
        _primary_motor.GetConfigurator().Apply(motor_config);
        _secondary_motor.GetConfigurator().Apply(motor_config);
        _secondary_motor.SetControl(controls::Follower{_primary_motor.GetDeviceID(), false});
        _trapezoid_timer.Start();
}

// This method will be called once per scheduler run
void ElevatorSubsystem::Periodic() {
    switch (_elevator_state)
    {
    case home:
        volt_t feed_forward_output = _elevator_feed_forward.Calculate(HOME_VELOCITY);
        _primary_motor.SetVoltage(feed_forward_output);
        if (_HomeSensor() || _GetStalled() ) {
            SetPower(0);
            _primary_motor.SetPosition(0_t);
            _elevator_pid_controller.Reset();
            _elevator_state = ready;
        }
        
        break;
    case ready:
        auto current_state = _elevator_trapezoid.Calculate(_trapezoid_timer.Get(), _initial_state, _target_state);
        volt_t feed_forward_output = _elevator_feed_forward.Calculate(_GetElevatorVelocity(), meters_per_second_t{current_state.velocity});
        volt_t pid_output = volt_t{_elevator_pid_controller.Calculate(inch_t{_GetElevatorHeight()}.value(), inch_t{current_state.position}.value())};
        _primary_motor.SetVoltage(feed_forward_output+pid_output);
        
        break;
    case test:
        

        break;
    default:
        _elevator_state = home;
        break;
    }
}

void ElevatorSubsystem::SetHeight(inch_t height) {
    if (height != _target_state.position) {
        _target_state.position = height;
        _target_state.velocity = 0_fps;
        _initial_state.position = _GetElevatorHeight();
        _initial_state.velocity = _GetElevatorVelocity();
        _trapezoid_timer.Reset();
    }
}

bool ElevatorSubsystem::AtTargetHeight() {
    return math::abs(_target_state.position - _GetElevatorHeight()) < POSITION_TOLERANCE;
}

void ElevatorSubsystem::SetPower(double power) {
    if (_elevator_state == test) {
        _primary_motor.Set(power);
    }
}

void ElevatorSubsystem::SetTestMode(bool test_mode) {
    if (test_mode) {
        _elevator_state = test;
    } else if (_elevator_state == test) {
        _elevator_state = home;
        _elevator_pid_controller.Reset();
    }
}

bool ElevatorSubsystem::_HomeSensor() {
    return !_home_sensor.Get();
}

bool ElevatorSubsystem::_GetStalled() {
    return (_primary_motor.GetSupplyCurrent().GetValue()/(_primary_motor.GetMotorStallCurrent().GetValue()*_primary_motor.Get())) > STALL_LIMIT;
}

inch_t ElevatorSubsystem::_GetElevatorHeight() {
    return _primary_motor.GetPosition().GetValue() * ELEVATOR_RATIO;
}

feet_per_second_t ElevatorSubsystem::_GetElevatorVelocity() {
    return _primary_motor.GetVelocity().GetValue() * ELEVATOR_RATIO;
}