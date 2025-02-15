#ifndef AUTOMATIC_SCORE_CORAL_COMMAND_H
#define AUTOMATIC_SCORE_CORAL_COMMAND_H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/IntakeSubsystem.h"
#include "subsystems/PivotSubsystem.h"
#include "subsystems/DrivetrainSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"

#include "OI.h"

/**
 * Scores coral into the reef by setting the height of the elevator, extending the pivot, and running the intake
 */
class AutomaticScoreCoralCommand
    : public frc2::CommandHelper<frc2::Command, AutomaticScoreCoralCommand> {
	public:
		AutomaticScoreCoralCommand(
			DrivetrainSubsystem* drivetrain, 
			ElevatorSubsystem* elevator,
			IntakeSubsystem* intake, 
			PivotSubsystem* pivot,   
			Driver_Interface* oi
		);

		void Initialize() override;

		void Execute() override;

		void End(bool interrupted) override;

		bool IsFinished() override;

	private:
		enum State {wait, extend_elevator, extend_pivot, eject_piece, done};
		State _auto_score_coral_state = wait;

		DrivetrainSubsystem* _drivetrain;
		ElevatorSubsystem* _elevator;
		IntakeSubsystem* _intake;
		PivotSubsystem* _pivot;
		Driver_Interface* _oi;
};

#endif