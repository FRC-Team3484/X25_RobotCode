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
            int coral_high_sensor_di_ch,
            int coral_low_sensor_di_ch
        );
        void Periodic() override;

        /** 
         * Controls the roller power for the Algae
         * 
         * @param power The roller power for the Algae is between -1.0 and 1.0
        */
        void SetAlgaePower(double power);
        
        /** 
         * Controls the roller power for the Coral
         * 
         * @param power The roller power for the Coral is between -1.0 and 1.0
        */
        void SetCoralPower(double power); 

        /** 
         * Checks to see if the sensor is detecting the Algae 
         * 
         * @return Returns false if the sensor detects the Alage
        */
        bool HasAlgae(); 

        /** 
         * Checks to see if the sensor is detecting the Coral
         * 
         * @return Returns false if the sensor detects the Coral
        */
        bool CoralHigh();

        bool CoralLow();

        /**
         * 
        */
        void PrintTestInfo();
        
    private:
        ctre::phoenix6::hardware::TalonFX _intake_motor_1;
        ctre::phoenix6::hardware::TalonFX _intake_motor_2;
        frc::DigitalInput _algae_sensor;
        frc::DigitalInput _coral_high_sensor;
        frc::DigitalInput _coral_low_sensor;
};

#endif