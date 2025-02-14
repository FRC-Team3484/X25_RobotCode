// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#ifndef AUTOMATIC_SCORE_CORAL_COMMAND_H
#define AUTOMATIC_SCORE_CORAL_COMMAND_H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/IntakeSubsystem.h"
#include "subsystems/PivotSubsystem.h"
#include "subsystems/DrivetrainSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"

#include "OI.h"

#include "StowArmCommand.h"

/**
* An example command.
*
* <p>Note that this extends CommandHelper, rather extending Command
* directly; this is crucially important, or else the decorator functions in
* Command will *not* work!
*/
class AutomaticScoreCoralCommand
    : public frc2::CommandHelper<frc2::Command, AutomaticScoreCoralCommand> {
	public:
		/* You should consider using the more terse Command factories API instead
		* https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
		*/
		AutomaticScoreCoralCommand(
			IntakeSubsystem* intake_subsystem,
			PivotSubsystem* pivot_subsystem,
			DrivetrainSubsystem* drivetrain_subsystem,
			ElevatorSubsystem* elevator_subsystem,
			Driver_Interface* oi
		);

		void Initialize() override;

		void Execute() override;

		void End(bool interrupted) override;

		bool IsFinished() override;

	private:
		enum State {wait, extend_elevator, extend_pivot, eject_piece, retract_pivot, retract_elevator};
		State _auto_score_coral_state = wait;

		IntakeSubsystem* _intake_subsystem;
		PivotSubsystem* _pivot_subsystem;
		DrivetrainSubsystem* _drivetrain_subsystem;
		ElevatorSubsystem* _elevator_subsystem;
		Driver_Interface* _oi;
};

#endif