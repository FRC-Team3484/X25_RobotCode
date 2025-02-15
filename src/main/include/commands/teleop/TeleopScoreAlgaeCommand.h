#ifndef TELEOP_SCORE_ALGAE_COMMAND_H
#define TELEOP_SCORE_ALGAE_COMMAND_H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/IntakeSubsystem.h"
#include "subsystems/PivotSubsystem.h"
#include "subsystems/DrivetrainSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"

#include "OI.h"

class TeleopScoreAlgaeCommand
	: public frc2::CommandHelper<frc2::Command, TeleopScoreAlgaeCommand> {
	public:
		TeleopScoreAlgaeCommand(
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
		enum State {wait, extend_elevator, extend_pivot, eject_piece, done};
		State _auto_score_algae_state = wait;

		DrivetrainSubsystem* _drivetrain;
		ElevatorSubsystem* _elevator;
		IntakeSubsystem* _intake;
		PivotSubsystem* _pivot;
		Operator_Interface* _oi;
};


#endif