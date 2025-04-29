#ifndef AUTON_SCORE_CORAL_COMMAND_H
#define AUTON_SCORE_CORAL_COMMAND_H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/IntakeSubsystem.h"
#include "subsystems/PivotSubsystem.h"
#include "subsystems/DrivetrainSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"

#include <frc/Timer.h>

#include "Datatypes.h"

class AutonScoreCoralCommand
: public frc2::CommandHelper<frc2::Command, AutonScoreCoralCommand> {
 public:
  AutonScoreCoralCommand(
    DrivetrainSubsystem* drivetrain, 
    ElevatorSubsystem* elevator,
    IntakeSubsystem* intake, 
    PivotSubsystem* pivot,
    AutonLevel::AutonLevel reef_level
  );

        void Initialize() override;

        void Execute() override;

        void End(bool interrupted) override;

        bool IsFinished() override;

    private:
        enum State {wait, traveling_pivot, extend_elevator, extend_pivot, eject_piece, done};
		State _auton_score_coral_state = wait;

    DrivetrainSubsystem* _drivetrain;
		ElevatorSubsystem* _elevator;
		IntakeSubsystem* _intake;
		PivotSubsystem* _pivot;

    AutonLevel::AutonLevel _reef_level;

    frc::Timer _eject_timer;
};

#endif