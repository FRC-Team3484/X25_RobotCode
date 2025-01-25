#include "subsystems/IntakeSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

IntakeSubsystem::IntakeSubsystem(
    int _motor_can_id,
    int _algae_sensor_di_ch,
    int _coral_high_sensor_di_ch,
    int _coral_low_sensor_di_ch
    ) : 
        _intake_motor{_motor_can_id},
        _algae_sensor{_algae_sensor_di_ch},
        _coral_high_sensor{_coral_high_sensor_di_ch},
        _coral_low_sensor{_coral_low_sensor_di_ch}
    {

};

void IntakeSubsystem::Periodic() {}

void IntakeSubsystem::SetPower(double power) {
    _intake_motor.Set(power);
}

bool IntakeSubsystem::HasAlgae() {
    return !_algae_sensor.Get();
}

bool IntakeSubsystem::CoralHigh() {
    return !_coral_high_sensor.Get();
}

bool IntakeSubsystem::CoralLow() {
    return !_coral_low_sensor.Get();
}

void IntakeSubsystem::PrintTestInfo() {
    frc::SmartDashboard::PutBoolean("Have Algae", HasAlgae());
    frc::SmartDashboard::PutBoolean("Have Coral High", CoralHigh());
    frc::SmartDashboard::PutBoolean("Have Coral Low", CoralLow());
}