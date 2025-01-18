#include "subsystems/PivotSubsystem.h"

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

void PivotSubsystem::SetPivotAngle(units::degree_t angle) {

}

units::degree_t PivotSubsystem::GetPivotAngle() {

}

bool PivotSubsystem::AtTargetPosition() {

}