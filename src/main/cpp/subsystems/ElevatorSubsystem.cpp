#include "subsystems/ElevatorSubsystem.h"
#include <units/math.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace ctre::phoenix6;
using namespace ElevatorConstants;
using namespace units;

ElevatorSubsystem::ElevatorSubsystem(
    int primary_motor_can_id,
    int secondary_motor_can_id,
    int home_sensor_di_ch,
    int brake_servo_port,
    SC::SC_PIDConstants elevator_pidc,
    feet_per_second_t max_velocity,
    feet_per_second_squared_t max_acceleration,
    SC::SC_LinearFeedForward feed_forward_constants
    ) : 
    _primary_motor(primary_motor_can_id),
    _secondary_motor(secondary_motor_can_id),
    _home_sensor(home_sensor_di_ch),
    _brake_servo{brake_servo_port},
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
        _secondary_motor.SetControl(controls::Follower{_primary_motor.GetDeviceID(), MIRROR_MOTORS});
        _trapezoid_timer.Start();

        _elevator_state = home;
}

void ElevatorSubsystem::Periodic() {
    volt_t feed_forward_output;
    frc::TrapezoidProfile<units::feet>::State current_state;
    volt_t pid_output;
    
    if (_HomeSensor()) {
        _SetPosition(HOME_POSITION);
    }
    if (!_isHomed){
        _elevator_state = home;
    }
    
    switch (_elevator_state) {
        case home:
            // Set the elevator to the home position
            feed_forward_output = _elevator_feed_forward.Calculate(HOME_VELOCITY);
            _primary_motor.SetVoltage(feed_forward_output);
            if (_HomeSensor() || _GetStalled() ) {
                SetPower(0);
                _SetPosition(HOME_POSITION);
                _elevator_pid_controller.Reset();
                _isHomed = true;
                _elevator_state = ready;
                _SetPosition(HOME_POSITION);
            }
            break;
        case ready:
            // Set the elevator to the target position given by SetHeight()
            if ((math::abs(_target_state.position - HOME_POSITION) < POSITION_TOLERANCE) && _HomeSensor()) {
                _elevator_pid_controller.Reset();
                _previous_elevator_velocity = 0_mps;
                SetPower(0);
                _SetPosition(HOME_POSITION);
            } else {
                current_state = _elevator_trapezoid.Calculate(_trapezoid_timer.Get(), _initial_state, _target_state);
                feed_forward_output = _elevator_feed_forward.Calculate(_previous_elevator_velocity, meters_per_second_t{current_state.velocity});
                pid_output = volt_t{_elevator_pid_controller.Calculate(inch_t{_GetElevatorHeight()}.value(), inch_t{current_state.position}.value())};
                _primary_motor.SetVoltage(feed_forward_output+pid_output);
                _previous_elevator_velocity = current_state.velocity;
            }
            break;
        case test:
            break;
        default:
            _elevator_state = home;
            break;
    }
    PrintTestInfo();
    // If we're climbing, engage the elevator brake
    if (_climbing) {
        _brake_servo.Set(RATCHET_ENGAGED);
    } else {
        _brake_servo.Set(RATCHET_DISENGAGED);
    }
}

void ElevatorSubsystem::SetHeight(inch_t height) {
    if (height != _target_state.position) {
        if (height == HOME_POSITION && _target_state.position == CLIMB_HEIGHT) {
            _climbing = true;
        } else {
            _climbing = false;
        }
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

bool ElevatorSubsystem::AtSafeStowPosition() {
    return math::abs(_target_state.position - _GetElevatorHeight()) < SAFE_STOW_POSITION;
}

void ElevatorSubsystem::SetPower(double power) {
    _primary_motor.Set(power);
}

void ElevatorSubsystem::SetTestMode(bool test_mode) {
    if (test_mode) {
        _elevator_state = test;
    } else if (_elevator_state == test) {
        _elevator_state = home;
        _elevator_pid_controller.Reset();
        _SetPosition(HOME_POSITION);
    }
}

void ElevatorSubsystem::PrintTestInfo() {
    frc::SmartDashboard::PutNumber("Elevator Height (in)", _GetElevatorHeight().value());
    frc::SmartDashboard::PutNumber("Elevator Stall", _GetStallPercentage());
    frc::SmartDashboard::PutBoolean("Elevator Home Sensor", _HomeSensor());
    frc::SmartDashboard::PutNumber("Elevator Voltage Demand", _primary_motor.GetMotorVoltage().GetValue().value());
    frc::SmartDashboard::PutNumber("Elevator Velocity", _GetElevatorVelocity().value()*12.0);
}

bool ElevatorSubsystem::_HomeSensor() {
    return !_home_sensor.Get();
}

bool ElevatorSubsystem::_GetStalled() {
    return _GetStallPercentage() > STALL_LIMIT;
}

double ElevatorSubsystem::_GetStallPercentage() {
    if (abs(_primary_motor.Get()) > STALL_TRIGGER) {
        return (_primary_motor.GetSupplyCurrent().GetValue()/(_primary_motor.GetMotorStallCurrent().GetValue()*abs(_primary_motor.Get())));
    } else {
        return 0;
    }
}

inch_t ElevatorSubsystem::_GetElevatorHeight() {
    return (_primary_motor.GetPosition().GetValue() * ELEVATOR_RATIO) + _offset;
}

feet_per_second_t ElevatorSubsystem::_GetElevatorVelocity() {
    return _primary_motor.GetRotorVelocity().GetValue() * ELEVATOR_RATIO;
}

void ElevatorSubsystem::_SetPosition(inch_t offset) {
    _offset = offset - (_primary_motor.GetPosition().GetValue() * ELEVATOR_RATIO);
}

void ElevatorSubsystem::SetStateToHome() {
    _elevator_state = home;
}