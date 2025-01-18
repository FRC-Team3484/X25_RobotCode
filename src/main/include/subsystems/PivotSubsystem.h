#ifndef PIVOT_SUBSYSTEM_H
#define PIVOT_SUBSYSTEM_H

#include <Constants.h>
#include <units/angle.h>

#include <frc2/command/SubsystemBase.h>
#include <frc/DigitalInput.h>
#include <ctre/phoenix6/TalonFX.hpp>

class PivotSubsystem : public frc2::SubsystemBase {
    public:
        PivotSubsystem(
            int pivot_motor_can_id,
            int pivot_home_di_ch
        );

        void Periodic() override;

        void SetPivotAngle(units::degree_t angle);
        units::degree_t GetPivotAngle();
        bool AtTargetPosition();

    private:
        ctre::phoenix6::hardware::TalonFX _pivot_motor;
        frc::DigitalInput _pivot_home;
};

#endif