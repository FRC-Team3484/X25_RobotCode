#ifndef INTAKE_SUBSYSTEM_H
#define INTAKE_SUBSYSTEM_H

#include "Constants.h"
#include "frc/DigitalInput.h"

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFXS.hpp>

/**
 * Subsystem class to handle control for the Intake. 
 * This class has functions to check if the intake has 
 * the Coral or the Algae, and sets motor power accordingly.
 */
class IntakeSubsystem : public frc2::SubsystemBase {
    public:
        /**
         * Creates an instance of the intake subsystem, which controls the intake, allowing the robot to pickup and eject coral and algae
         * 
         * @param motor_can_id The CAN ID for the motor
         * @param algae_top_sensor_di_ch The ID for the top sensor which detects the algae
         * @param algae_bottom_sensor_di_ch The ID for the bottom sensor which detects the algae
         * @param coral_high_sensor_di_ch The ID for the higher sensor which detects the coral
         * @param coral_low_sensor_di_ch The ID for the lower sensor which detects the coral
         */
        IntakeSubsystem(
            int motor_can_id,
            int algae_top_sensor_di_ch,
            int algae_bottom_sensor_di_ch,
            int coral_high_sensor_di_ch,
            int coral_low_sensor_di_ch
        );
        void Periodic() override;

        /** 
         * Controls the roller power for the intake (algae)
         * 
         * @param power The roller power is between -1.0 and 1.0
        */
        void SetPower(double power);

        /** 
         * Checks to see if the top sensor is detecting the Algae 
         * 
         * @return Returns true if the sensor detects the Alage
        */
        bool HasAlgae(); 

        /** 
         * Checks to see if the higher sensor is detecting the Coral
         * 
         * @return Returns true if the sensor detects the Coral
        */
        bool CoralHigh();

        /**
         * Checks to see if the lower sensor is detecting the Coral
         * 
         * @return Returns true if the sensor detects Coral
         */
        bool CoralLow();

        /**
         *  Checks to see if the intake has the Coral
         * 
         * @return Returns true if the intake has the Coral 
         */
        bool HasCoral();

        /**
         * Prints the test info for this subsystem to SmartDashboard 
        */
        void PrintTestInfo();
        
    private:
        ctre::phoenix6::hardware::TalonFXS _intake_motor;
        frc::DigitalInput _algae_top_sensor;
        frc::DigitalInput _algae_bottom_sensor;
        frc::DigitalInput _coral_high_sensor;
        frc::DigitalInput _coral_low_sensor;
};

#endif