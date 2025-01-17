#include "subsystems/IntakeSubsystem.h"

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

}

bool IntakeSubsystem::HasAlgae() {

}

bool IntakeSubsystem::HasCoral() {

}