#ifndef AUTON_SCORE_CORAL_COMMAND_H
#define AUTON_SCORE_CORAL_COMMAND_H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/IntakeSubsystem.h"
#include "subsystems/PivotSubsystem.h"
#include "subsystems/DrivetrainSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"

class AutonScoreCoralCommand
: public frc2::CommandHelper<frc2::Command, AutonScoreCoralCommand> {
 public:
  AutonScoreCoralCommand(
    DrivetrainSubsystem* drivetrain, 
    ElevatorSubsystem* elevator,
    IntakeSubsystem* intake, 
    PivotSubsystem* pivot,
    std::string reef_level
  );

        void Initialize() override;

        void Execute() override;

        void End(bool interrupted) override;

        bool IsFinished() override;

    private:
        enum State {wait, extend_elevator, extend_pivot, eject_piece, done};
		State _auton_score_coral_state = wait;

        DrivetrainSubsystem* _drivetrain;
		ElevatorSubsystem* _elevator;
		IntakeSubsystem* _intake;
		PivotSubsystem* _pivot;

    std::string _reef_level;
};

#endif