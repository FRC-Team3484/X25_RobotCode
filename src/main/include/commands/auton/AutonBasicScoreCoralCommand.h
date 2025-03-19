#ifndef AUTON_BASIC_SCORE_CORAL_COMMAND_H
#define AUTON_BASIC_SCORE_CORAL_COMMAND_H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/IntakeSubsystem.h"
#include "subsystems/PivotSubsystem.h"
#include "subsystems/DrivetrainSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"

class AutonBasicScoreCoralCommand
    : public frc2::CommandHelper<frc2::Command, AutonBasicScoreCoralCommand> {
    public:
        /**
         * Scores coral into the reef in auton
         * 
         * @param drivetrain A pointer to the drivetrain subsystem
         * @param elevator A pointer to the elevator subsystem
         * @param intake A pointer to the intake subsystem
         * @param pivot A pointer to the pivot subsystem
         */
        AutonBasicScoreCoralCommand(
            DrivetrainSubsystem* drivetrain, 
            ElevatorSubsystem* elevator,
            IntakeSubsystem* intake, 
            PivotSubsystem* pivot
        );

        void Initialize() override;

        void Execute() override;

        void End(bool interrupted) override;

        bool IsFinished() override;

    private:
        enum State {traveling_pivot, extend_elevator, extend_pivot, eject_piece, done};
		State _auton_score_coral_state = traveling_pivot;

        DrivetrainSubsystem* _drivetrain;
		ElevatorSubsystem* _elevator;
		IntakeSubsystem* _intake;
		PivotSubsystem* _pivot;
};

#endif