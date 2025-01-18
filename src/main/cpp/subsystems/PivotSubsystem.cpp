#include "subsystems/PivotSubsystem.h"
#include <units/angle.h>
#include <units/math.h>


using namespace units;
PivotSubsystem::PivotSubsystem(
    int pivot_motor_can_id,
    int pivot_home_di_ch
    ) :
        _pivot_motor{pivot_motor_can_id},
        _pivot_home{pivot_home_di_ch}
    {

}

void PivotSubsystem::Periodic() {

}

void PivotSubsystem::SetPivotAngle(degree_t angle) {
    if (angle != _target_state.position) {
        _target_state.position = angle;
        _target_state.velocity = 0_deg_per_s;
        _intitial_state.position = GetPivotAngle();
        _intitial_state.velocity = GetPivotVelocity();
        _trapezoid_timer.Reset();
    }
}

degree_t PivotSubsystem::GetPivotAngle() {
    return _pivot_motor.GetPosition().GetValue(); // Type casts revolutions into degrees
}
degrees_per_second_t PivotSubsystem::GetPivotVelocity(){
    return _pivot_motor.GetVelocity().GetValue();
}

bool PivotSubsystem::AtTargetPosition() {
    return math::abs(_target_state.position - GetPivotAngle()) < Pivot::ANGLE_TOLERANCE;
}