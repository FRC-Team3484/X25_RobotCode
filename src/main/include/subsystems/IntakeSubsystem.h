#ifndef INTAKE_SUBSYSTEM_H
#define INTAKE_SUBSYSTEM_H

#include "Constants.h"
#include "frc/DigitalInput.h"

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>

/**
 * Subsystem class to handle control for the Intake. 
 * This class has functions to check if the intake has 
 * the Coral or the Algae, and sets motor power accordingly.
 */

class IntakeSubsystem : public frc2::SubsystemBase {
    public:
        IntakeSubsystem(
            int motor_one_can_id,
            int motor_two_can_id,
            int algae_sensor_di_ch,
            int coral_sensor_di_ch
        );
        void Periodic() override;
        void SetRollerPower(double power); /** Controls the roller power */
        bool HasAlgae(); /** Checks to see if the sensor is detecting the Algae */
        bool HasCoral(); /** Checks to see if the sensor is detecting the Coral */
    private:
        ctre::phoenix6::hardware::TalonFX _intake_motor_1;
        ctre::phoenix6::hardware::TalonFX _intake_motor_2;
        frc::DigitalInput _algae_sensor;
        frc::DigitalInput _coral_sensor;
};

#endif