#ifndef PIVOT_SUBSYSTEM_H
#define PIVOT_SUBSYSTEM_H

#include <Constants.h>

#include <frc2/command/SubsystemBase.h>
#include <units/angle.h>

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
};

#endif