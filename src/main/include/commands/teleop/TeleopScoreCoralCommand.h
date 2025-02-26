#ifndef TELEOP_SCORE_CORAL_COMMAND_H
#define TELEOP_SCORE_CORAL_COMMAND_H

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
class TeleopScoreCoralCommand
    : public frc2::CommandHelper<frc2::Command, TeleopScoreCoralCommand> {
	public:
		/**
         * 
         * @param pivot A pointer to the pivot subsystem
         * @param elevator A pointer to the elevator interface
		 * @param intake A pointer to the intake subsystem
		 * @param drivetrain A pointer to the drivetrain subsystem
		 * @param oi A pointer to the operator interface 
         */
		TeleopScoreCoralCommand(
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
		State _auto_score_coral_state = wait;

		DrivetrainSubsystem* _drivetrain;
		ElevatorSubsystem* _elevator;
		IntakeSubsystem* _intake;
		PivotSubsystem* _pivot;
		Operator_Interface* _oi;
};

#endif