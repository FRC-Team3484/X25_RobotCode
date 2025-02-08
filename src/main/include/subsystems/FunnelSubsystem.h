// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#ifndef FUNNEL_SUBSYSTEM_H
#define FUNNEL_SUBSYSTEM_H

#include "Constants.h"
#include <frc/DigitalInput.h>

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>

class FunnelSubsystem : public frc2::SubsystemBase {
    public:
        FunnelSubsystem(
            int motor_can_id,
            int coral_sensor_di_ch
        );

        /**
         * Will be called periodically whenever the CommandScheduler runs.
         */
        void Periodic() override;

        void SetPower(double power);

        bool hasCoral();

        void PrintTestInfo();

    private:
        ctre::phoenix6::hardware::TalonFX _funnel_motor;
        frc::DigitalInput _coral_sensor;

};

#endif