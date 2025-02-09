#include "subsystems/FunnelSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

FunnelSubsystem::FunnelSubsystem(
    int motor_can_id,
    int coral_sensor_di_ch
    ) :
        _funnel_motor{motor_can_id},
        _coral_sensor{coral_sensor_di_ch}
    {
};

void FunnelSubsystem::Periodic() {}

void FunnelSubsystem::SetPower(double power) {
    _funnel_motor.Set(power);
}

bool FunnelSubsystem::HasCoral() {
    return !_coral_sensor.Get();
}

void FunnelSubsystem::PrintTestInfo() {
    frc::SmartDashboard::PutBoolean("Has Coral", HasCoral());
}