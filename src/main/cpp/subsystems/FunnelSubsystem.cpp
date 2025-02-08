// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/FunnelSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

FunnelSubsystem::FunnelSubsystem(
    int motor_can_id,
    int coral_sensor_di_ch
    ) :
        _funnel_motor(motor_can_id),
        _coral_sensor(coral_sensor_di_ch)
    {

}

// This method will be called once per scheduler run
void FunnelSubsystem::Periodic() {}

void FunnelSubsystem::SetPower(double power) {
    _funnel_motor.Set(power);
}

bool FunnelSubsystem::hasCoral() {
    return !_coral_sensor.Get();
}

void FunnelSubsystem::PrintTestInfo() {
    frc::SmartDashboard::PutBoolean("Has Coral", hasCoral());
}


