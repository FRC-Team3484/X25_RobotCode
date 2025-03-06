#include "subsystems/ElevatorSubsystem.h"
#include <units/math.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc2/command/Commands.h>

using namespace ctre::phoenix6;
using namespace ElevatorConstants;
using namespace units;

ElevatorSubsystem::ElevatorSubsystem(
    int primary_motor_can_id,
    int secondary_motor_can_id,
    int home_sensor_di_ch
    ) : 
    _primary_motor(primary_motor_can_id),
    _secondary_motor(secondary_motor_can_id),
    _home_sensor(home_sensor_di_ch)

    {
        configs::TalonFXConfiguration motor_config{};
        motor_config.MotorOutput.Inverted = INVERT_MOTORS;
        motor_config.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
        _primary_motor.GetConfigurator().Apply(motor_config);
        motor_config.MotorOutput.Inverted = INVERT_MOTORS ^ MIRROR_MOTORS;
        _secondary_motor.GetConfigurator().Apply(motor_config);
        _secondary_motor.SetControl(controls::Follower{_primary_motor.GetDeviceID(), false});
}

frc2::CommandPtr ElevatorSubsystem::PseudoMoveCommand(std::function<double()> power) {
    return frc2::cmd::Run([this, power] {
        SetPower(power());
    }, 
    {this});
}

frc2::CommandPtr ElevatorSubsystem::SysIdQuasistatic(frc2::sysid::Direction direction) {
    return _sysid_routine.Quasistatic(direction);
}

frc2::CommandPtr ElevatorSubsystem::SysIdDynamic(frc2::sysid::Direction direction) {
    return _sysid_routine.Dynamic(direction);
}




void ElevatorSubsystem::Periodic() {
    PrintTestInfo();
}

void ElevatorSubsystem::SetVoltage(units::volt_t voltage) {
    _primary_motor.SetVoltage(voltage);
}

void ElevatorSubsystem::SetPower(double power) {
    _primary_motor.Set(power);
}

void ElevatorSubsystem::PrintTestInfo() {
    frc::SmartDashboard::PutNumber("Elevator Height (in)", _GetElevatorHeight().value());
    frc::SmartDashboard::PutNumber("Elevator Stall", _GetStallPercentage());
    frc::SmartDashboard::PutBoolean("Elevator Home Sensor", _HomeSensor());
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
    return _primary_motor.GetPosition().GetValue() * ELEVATOR_RATIO;
}

feet_per_second_t ElevatorSubsystem::_GetElevatorVelocity() {
    return _primary_motor.GetVelocity().GetValue() * ELEVATOR_RATIO;
}
