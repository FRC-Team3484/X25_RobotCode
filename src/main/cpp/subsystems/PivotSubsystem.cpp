#include "subsystems/PivotSubsystem.h"
#include <units/angle.h>
#include <units/math.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace units;
using namespace PivotConstants;
using namespace ctre::phoenix6;

PivotSubsystem::PivotSubsystem(
    int pivot_motor_can_id,
    int pivot_home_di_ch,
    SC::SC_PIDConstants pivot_pidc,
    radians_per_second_t max_velocity,
    radians_per_second_squared_t max_acceleration,
    SC::SC_AngularFeedForward feed_forward_constants
    ) :
    _pivot_motor{pivot_motor_can_id},
    _pivot_home{pivot_home_di_ch},
    _pivot_pid_controller{pivot_pidc.Kp, pivot_pidc.Ki, pivot_pidc.Kd},
    _pivot_trapezoid{{max_velocity, max_acceleration}},
    _pivot_feed_forward{
        feed_forward_constants.S,
        feed_forward_constants.G,
        feed_forward_constants.V,
        feed_forward_constants.A
    }
    {
        configs::TalonFXConfiguration motor_config{};
        motor_config.MotorOutput.Inverted = INVERT_MOTOR;
        motor_config.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
        _pivot_motor.GetConfigurator().Apply(motor_config);
        _trapezoid_timer.Start();
        
        frc::SmartDashboard::PutBoolean("En Pivot PID Tuning", false);
        frc::SmartDashboard::PutNumber("Kp_Pivot", pivot_pidc.Kp);
        frc::SmartDashboard::PutNumber("Ki_Pivot", pivot_pidc.Ki);
        frc::SmartDashboard::PutNumber("Kd_Pivot", pivot_pidc.Kd);
}

void PivotSubsystem::Periodic() {
    volt_t feed_forward_output;
    frc::TrapezoidProfile<deg>::State current_state;
    volt_t pid_output;

    if(frc::SmartDashboard::GetBoolean("En Pivot PID Tuning", false))
    {
        _pivot_pid_controller.SetPID(
            frc::SmartDashboard::GetNumber("Kp_Pivot", _pivot_pid_controller.GetP()),
            frc::SmartDashboard::GetNumber("Ki_Pivot", _pivot_pid_controller.GetI()),
            frc::SmartDashboard::GetNumber("Kd_Pivot", _pivot_pid_controller.GetD())
        );
    }

    if (_HomeSensor()){
        _SetPivotAngle(HOME_POSITION);
    }

    switch(_pivot_state){
    case home:
        // Homes the pivot
        _pivot_motor.Set(HOME_POWER);
        if (_HomeSensor()||_GetStalled()){
            SetPower(0);
            _pivot_pid_controller.Reset();
            _pivot_state = ready;
            SetPivotAngle(HOME_POSITION);
        }
        break;
    case ready:
        // Sets the pisvot to the target angle given in SetPivotAngle()
        if (_target_state.position == HOME_POSITION && _HomeSensor()) {
            _pivot_pid_controller.Reset();
            _previous_pivot_velocity = 0_rad_per_s;
            SetPower(0);
            _SetPivotAngle(HOME_POSITION);
        } else {
            SetPower(0);
            current_state = _pivot_trapezoid.Calculate(_trapezoid_timer.Get(), _intitial_state, _target_state);
            feed_forward_output = _pivot_feed_forward.Calculate(radian_t{_GetPivotAngle()}, _previous_pivot_velocity, radians_per_second_t{current_state.velocity});
            pid_output = volt_t{_pivot_pid_controller.Calculate(degree_t{_GetPivotAngle()}.value(), degree_t{current_state.position}.value())};
            _pivot_motor.SetVoltage(feed_forward_output+pid_output);
            _previous_pivot_velocity = current_state.velocity;
        }
        break;
    case test:
        break;
    default:
        _pivot_state=home;
        break;
    }
    PrintTestInfo();
}

void PivotSubsystem::SetPivotAngle(degree_t angle) {
    if (angle != _target_state.position) {
        _target_state.position = angle;
        _target_state.velocity = 0_deg_per_s;
        _intitial_state.position = _GetPivotAngle();
        _intitial_state.velocity = _GetPivotVelocity();
        _trapezoid_timer.Reset();
    }
}

double PivotSubsystem::_GetStallPercentage() {
    if (abs(_pivot_motor.Get()) > STALL_TRIGGER) {
        return (_pivot_motor.GetSupplyCurrent().GetValue()/(_pivot_motor.GetMotorStallCurrent().GetValue()*abs(_pivot_motor.Get())));
    } else {
        return 0;
    }
}

bool PivotSubsystem::_GetStalled() {
    return _GetStallPercentage()>STALL_LIMIT;
}

bool PivotSubsystem::_HomeSensor() {
    return !_pivot_home.Get();
}

void PivotSubsystem::PrintTestInfo() {
    frc::SmartDashboard::PutNumber("Pivot Angle (deg)", _GetPivotAngle().value());
    frc::SmartDashboard::PutNumber("Pivot Stall", _GetStallPercentage());
    frc::SmartDashboard::PutBoolean("Pivot Home Sensor", _HomeSensor());
    frc::SmartDashboard::PutNumber("Pivot Voltage Demand", _pivot_motor.GetMotorVoltage().GetValue().value());
}

void PivotSubsystem::SetTestMode(bool test_mode) {
    if (test_mode) {
        _pivot_state = test;
    } else if (_pivot_state == test) {
        _pivot_state = home;
        _pivot_pid_controller.Reset();
        SetPivotAngle(HOME_POSITION);
    }
}

void PivotSubsystem::SetPower(double power) {
    _pivot_motor.Set(power);
}

bool PivotSubsystem::AtTargetPosition() {
    return math::abs(_target_state.position - _GetPivotAngle()) < ANGLE_TOLERANCE;
}

degree_t PivotSubsystem::_GetPivotAngle() {
    return (_pivot_motor.GetPosition().GetValue() / GEAR_RATIO) + _offset; // Type casts revolutions into degrees
}

degrees_per_second_t PivotSubsystem::_GetPivotVelocity(){
    return _pivot_motor.GetVelocity().GetValue() / GEAR_RATIO;
}

void PivotSubsystem::_SetPivotAngle(degree_t angle) {
    if (_target_state.position == HOME_POSITION){ 
        _offset = 90_deg;
    } else if (_HomeSensor()){
        _offset = HOME_POSITION - /*angle*/ - (_pivot_motor.GetPosition().GetValue() / GEAR_RATIO);
    }
}
