#include "subsystems/PivotSubsystem.h"
#include <units/angle.h>
#include <units/math.h>
#include <frc/smartdashboard/SmartDashboard.h>


using namespace units;
using namespace PivotConstants;
using namespace ctre::phoenix6;

PivotSubsystem::PivotSubsystem(
    int pivot_motor_can_id,
    int pivot_home_di_ch
    ) :
    _pivot_motor{pivot_motor_can_id},
    _pivot_home{pivot_home_di_ch}
    {
        configs::TalonFXConfiguration motor_config{};
        motor_config.MotorOutput.Inverted = INVERT_MOTOR;
        motor_config.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
        _pivot_motor.GetConfigurator().Apply(motor_config);
}

frc2::CommandPtr PivotSubsystem::PseudoMoveCommand(std::function<double()> power) {
    return frc2::cmd::Run([this, power] {
        SetPower(power());
        fmt::println("Pivot Moving: {}", power());
    }, 
    {this});
}

frc2::CommandPtr PivotSubsystem::SysIdQuasistatic(frc2::sysid::Direction direction) {
    return _sysid_routine.Quasistatic(direction);
}

frc2::CommandPtr PivotSubsystem::SysIdDynamic(frc2::sysid::Direction direction) {
    return _sysid_routine.Dynamic(direction);

}
void PivotSubsystem::Periodic() {
    if (_HomeSensor()){
            _SetPivotAngle(HOME_POSITION);
    }
    PrintTestInfo();
}


degree_t PivotSubsystem::_GetPivotAngle() {
        return (_pivot_motor.GetPosition().GetValue() / GEAR_RATIO) + _offset; // Type casts revolutions into degrees
}

void PivotSubsystem::_SetPivotAngle(degree_t angle) {
    _offset = angle - (_pivot_motor.GetPosition().GetValue() / GEAR_RATIO);
}

degrees_per_second_t PivotSubsystem::_GetPivotVelocity(){
    return _pivot_motor.GetVelocity().GetValue() / GEAR_RATIO;
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
}


void PivotSubsystem::SetPower(double power) {
        _pivot_motor.Set(power);
}
