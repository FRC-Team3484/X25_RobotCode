#ifndef TESTPIVOTCOMMAND_H
#define TESTPIVOTCOMMAND_H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/PivotSubsystem.h"
#include "Constants.h"
#include "OI.h"

class TestPivotCommand
	: public frc2::CommandHelper<frc2::Command, TestPivotCommand> {
	public:
		/**
		 * Allows the pivot to be tested using the controller buttons
		 * 
		 * @param pivot_subsystem A pointer to the pivot subsystem
		 * @param testing_interface A pointer to the testing interface
		*/
		TestPivotCommand(
			PivotSubsystem* pivot_subsystem,
			Testing_Interface* testing_interface
		);

		void Initialize() override;

		void Execute() override;

		void End(bool interrupted) override;

		bool IsFinished() override;

	private:
		PivotSubsystem* _pivot_subsystem;
		Testing_Interface* _testing_interface;
};
#endif