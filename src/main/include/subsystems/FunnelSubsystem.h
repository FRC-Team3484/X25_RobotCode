#ifndef FUNNEL_SUBSYSTEM_H
#define FUNNEL_SUBSYSTEM_H

#include "Constants.h"
#include <frc/DigitalInput.h>

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>

/**
 * The funnel subsystem sets the power of the indexer on the robot, helping to guide the Coral into the intake
 */
class FunnelSubsystem : public frc2::SubsystemBase {
    public:
        /**
         * Creates an instance of the funnel subsystem
         * 
         * @param motor_can_id The CAN ID for the motor
         * @param coral_sensor_di_ch The ID for the sensor which detects the Coral
         */
        FunnelSubsystem(
            int motor_can_id,
            int coral_sensor_di_ch
        );

        /**
         * Called every loop when the subsystem is active
         */
        void Periodic() override;

        /**
         * Sets the power of the funnel motor
         * 
         * @param power The power to set the funnel motor to
         */
        void SetPower(double power);

        /**
         * Checks to see if the sensor is detecting the Coral
         * 
         * @return Returns true if the sensor detects the Coral
         */
        bool HasCoral();

        /**
         * Prints test mode data to SmartDashboard
         */
        void PrintTestInfo();

    private:
        ctre::phoenix6::hardware::TalonFX _funnel_motor;
        frc::DigitalInput _coral_sensor;

};

#endif