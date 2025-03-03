#ifndef AUTON_FEEDER_CORAL_COMMAND_H
#define AUTON_FEEDER_CORAL_COMMAND_H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/IntakeSubsystem.h"
#include "subsystems/PivotSubsystem.h"
#include "subsystems/DrivetrainSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"
#include "subsystems/FunnelSubsystem.h"

class AutonFeederCoralCommand
    : public frc2::CommandHelper<frc2::Command, AutonFeederCoralCommand> {
    public:
        /**
         * Intakes coral from the feeder station in auton
         * 
         * @param drivetrain A pointer to the drivetrain subsystem
         * @param elevator A pointer to the elevator subsystem
         * @param intake A pointer to the intake subsystem    
         * @param pivot A pointer to the pivot subsystem 
         * @param funnel A pointer to the funnel subsystem
         */
        AutonFeederCoralCommand(
            DrivetrainSubsystem* drivetrain, 
            ElevatorSubsystem* elevator,
            IntakeSubsystem* intake, 
            PivotSubsystem* pivot,
            FunnelSubsystem* funnel
        );

        void Initialize() override;

        void Execute() override;

        void End(bool interrupted) override;

        bool IsFinished() override;

    private:
        enum State {wait, intake, done};
		State _auton_feeder_coral_state = wait;

		DrivetrainSubsystem* _drivetrain;
		ElevatorSubsystem* _elevator;
		IntakeSubsystem* _intake;
		PivotSubsystem* _pivot;
        FunnelSubsystem* _funnel;
};

#endif