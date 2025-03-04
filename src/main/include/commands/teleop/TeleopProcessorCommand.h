#ifndef TELEOP_PROCESSOR_COMMAND_H
#define TELEOP_PROCESSOR_COMMAND_H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/IntakeSubsystem.h"
#include "subsystems/PivotSubsystem.h"
#include "subsystems/DrivetrainSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"

#include "OI.h"

class TeleopProcessorCommand
    : public frc2::CommandHelper<frc2::Command, TeleopProcessorCommand> {
    public:
        /**
         * Scores an algae into the processor by extending the pivot and running the intakes
         * 
         * @param drivetrain A pointer to the drivetrain subsystem
         * @param elevator A pointer to the elevator subsystem
         * @param intake A pointer to the intake subsystem
         * @param pivot A pointer to the pivot subsystem
         * @param oi A pointer to the operator interface
         */
        explicit TeleopProcessorCommand(
            DrivetrainSubsystem* drivetrain,
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
        enum State {wait, traveling_pivot, extend_elevator, extend_pivot, eject_algae, done};
        State _auto_processor_state = wait;

        DrivetrainSubsystem* _drivetrain;
        ElevatorSubsystem* _elevator;
        IntakeSubsystem* _intake;
        PivotSubsystem* _pivot;
        Operator_Interface* _oi;
};

#endif