#include "subsystems/IntakeSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

IntakeSubsystem::IntakeSubsystem(
    int _motor_one_can_id,
    int _motor_two_can_id,
    int _algae_sensor_di_ch,
    int _coral_sensor_di_ch
) : 
    _intake_motor_1{_motor_one_can_id},
    _intake_motor_2{_motor_two_can_id},
    _algae_sensor{_algae_sensor_di_ch},
    _coral_sensor{_coral_sensor_di_ch}
{

};

void IntakeSubsystem::Periodic() {

}

void IntakeSubsystem::SetRollerPower(double power) {
    _intake_motor_1.Set(power);
    _intake_motor_2.Set(power);
}

bool IntakeSubsystem::HasAlgae() {
   return !_algae_sensor.Get();
}

bool IntakeSubsystem::HasCoral() {
    return !_coral_sensor.Get();
}

void IntakeSubsystem::PrintTestInfo() {
    frc::SmartDashboard::PutBoolean("Have Algae", HasAlgae());
    frc::SmartDashboard::PutBoolean("Have Coral", HasCoral());
}