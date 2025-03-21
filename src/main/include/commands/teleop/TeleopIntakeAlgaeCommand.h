#ifndef TELEOP_INTAKE_ALGAE_COMMAND_H
#define TELEOP_INTAKE_ALGAE_COMMAND_H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/IntakeSubsystem.h"
#include "subsystems/PivotSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"

#include "OI.h"

/**
 * Intakes an algae, by raising the elevator, extending the pivot, and running the intake
 */
class TeleopIntakeAlgaeCommand
    : public frc2::CommandHelper<frc2::Command, TeleopIntakeAlgaeCommand>
    {
    
    public:
        /**
         * Intakes an algae by raising the elevator, extending the pivot, and running the intake
         * 
         * @param elevator A pointer to the elevator subsystem
         * @param intake A pointer to the intake subsystem
         * @param pivot A pointer to the pivot subsystem
         * @param oi A pointer to the operator interface
         */
        explicit TeleopIntakeAlgaeCommand(
            ElevatorSubsystem* elevator, 
            IntakeSubsystem* intake, 
            PivotSubsystem* pivot, 
            Operator_Interface* oi
        );

        void Initialize() override;

        void Execute() override;

        void End(bool interrupted) override;

        bool IsFinished() override;

    private:
        ElevatorSubsystem* _elevator;
        IntakeSubsystem* _intake; 
        PivotSubsystem* _pivot;
        Operator_Interface* _oi;

        enum State {wait, traveling_pivot, extend_elevator, extend_pivot, intake, done};
        State _auto_intake_algae_state = wait;
};

#endif