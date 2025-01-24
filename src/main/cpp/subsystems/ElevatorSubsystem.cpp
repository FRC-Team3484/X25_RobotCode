// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ElevatorSubsystem.h"
#include <units/math.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>

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
    volt_t feed_forward_output;
    frc::TrapezoidProfile<units::feet>::State current_state;
    volt_t pid_output;
    switch (_elevator_state) {
        case home:
            feed_forward_output = _elevator_feed_forward.Calculate(HOME_VELOCITY);
            _primary_motor.SetVoltage(feed_forward_output);
            if (_HomeSensor() || _GetStalled() ) {
                SetPower(0);
                _primary_motor.SetPosition(0_tr);
                _elevator_pid_controller.Reset();
                _elevator_state = ready;
            }
            break;
            // the 'home' state is the state that the elevator is in when it is at the home position.
        case ready:
            current_state = _elevator_trapezoid.Calculate(_trapezoid_timer.Get(), _initial_state, _target_state);
            feed_forward_output = _elevator_feed_forward.Calculate(_GetElevatorVelocity(), meters_per_second_t{current_state.velocity});
            pid_output = volt_t{_elevator_pid_controller.Calculate(inch_t{_GetElevatorHeight()}.value(), inch_t{current_state.position}.value())};
            _primary_motor.SetVoltage(feed_forward_output+pid_output);
            
            break;
            // the 'ready' state is the state that the elevator is in when it is not at the home position.
        case test:
            PrintTestInfo();
            break;
        // the 'test' state is for debugging.
        default:
            _elevator_state = test;
            break;
    }
    // up above is a big switch statement that contains the different states of the elevator.
}

frc2::CommandPtr ElevatorSubsystem::PsuedoSetHeight(std::function<double()> height) {
    return frc2::cmd::Run([this, height] {SetHeight(inch_t{height()});});
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
// this function sets the height of the elevator

bool ElevatorSubsystem::AtTargetHeight() {
    return math::abs(_target_state.position - _GetElevatorHeight()) < POSITION_TOLERANCE;
}
// this function checks if the elevator is at the target height


void ElevatorSubsystem::SetPower(double power) {
    if (_elevator_state == test) {
        _primary_motor.Set(power);
    }
}
// this function sets the power of the elevator

void ElevatorSubsystem::SetTestMode(bool test_mode) {
    if (test_mode) {
        _elevator_state = test;
    } else if (_elevator_state == test) {
        _elevator_state = home;
        _elevator_pid_controller.Reset();
    }
}
// this function sets the elevator to test mode, or sets it to home mode if it is in test mode.

void ElevatorSubsystem::PrintTestInfo() {
    frc::SmartDashboard::PutNumber("Elevator Height (in)", _GetElevatorHeight().value());
    frc::SmartDashboard::PutNumber("Elevator Stall", _GetStallPercentage());
    frc::SmartDashboard::PutBoolean("Home Sensor", _HomeSensor());
}
// this function prints the test info for the elevator

bool ElevatorSubsystem::_HomeSensor() {
    return !_home_sensor.Get();
}
// this function checks if the home sensor is triggered

bool ElevatorSubsystem::_GetStalled() {
    return _GetStallPercentage() > STALL_LIMIT;
}
// this function checks if the elevator is stalled

double ElevatorSubsystem::_GetStallPercentage() {
    return (_primary_motor.GetSupplyCurrent().GetValue()/(_primary_motor.GetMotorStallCurrent().GetValue()*abs(_primary_motor.Get())));
}
// this function gets the percentage of the stall current of the motor, to check if the motor is stalled

inch_t ElevatorSubsystem::_GetElevatorHeight() {
    return _primary_motor.GetPosition().GetValue() * ELEVATOR_RATIO;
}
// this function gets the height of the elevator

feet_per_second_t ElevatorSubsystem::_GetElevatorVelocity() {
    return _primary_motor.GetVelocity().GetValue() * ELEVATOR_RATIO;
}
// this function gets the velocity of the elevator


// just in case. we were confused with the old code :), probably not needed.
    // frc2::sysid::SysIdRoutine m_sysIdRoutine{
    //     frc2::sysid::Config{std::nullopt, std::nullopt, std::nullopt,
    //                         std::nullopt},
    //     frc2::sysid::Mechanism{
    //         [this](units::volt_t driveVoltage) {
    //         _primary_motor.SetVoltage(driveVoltage);
    //         _secondary_motor.SetVoltage(driveVoltage);
            
    //     },
    //     [this](frc::sysid::SysIdRoutineLog* log) {
    //         log->Motor("drive-fl")
    //             .voltage(_drive_motor_FL.Get() *
    //                     frc::RobotController::GetBatteryVoltage())
    //             .position(units::meter_t{2_in * units::radian_t{360_deg * _drive_motor_FL.GetSelectedSensorPosition() / 2048.0 /(36000.0/5880.0)} / 1_rad})
    //             .velocity(units::meters_per_second_t{2_in * units::radians_per_second_t{_drive_motor_FL.GetSelectedSensorVelocity()*10.0*360_deg_per_s/2048.0/(36000.0/5880.0)} / 1_rad});
    //         log->Motor("drive-fr")
    //             .voltage(_drive_motor_FR.Get() *
    //                     frc::RobotController::GetBatteryVoltage())
    //             .position(units::meter_t{2_in * units::radian_t{360_deg * _drive_motor_FR.GetSelectedSensorPosition() / 2048.0 /(36000.0/5880.0)} / 1_rad})
    //             .velocity(units::meters_per_second_t{2_in * units::radians_per_second_t{_drive_motor_FR.GetSelectedSensorVelocity()*10.0*360_deg_per_s/2048.0/(36000.0/5880.0)} / 1_rad});
    //     },

    //     this}};

