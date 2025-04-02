#ifndef AUTON_GENERATOR_H
#define AUTON_GENERATOR_H

// Subsystems
#include "subsystems/DrivetrainSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"
#include "subsystems/PivotSubsystem.h"
#include "subsystems/IntakeSubsystem.h"

//Auton Commands
#include "commands/auton/AutonFeederCoralCommand.h"
#include "commands/auton/AutonScoreCoralCommand.h"
#include "commands/auton/AutonStopCommand.h"
#include "commands/teleop/StowArmCommand.h"

#include <frc2/command/Commands.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc/smartdashboard/SendableChooser.h>

#include "Datatypes.h"

class AutonGenerator {
    public:
        /**
         * Generates the autonomous commands for the robot
         * Also displays the dropdowns in ShuffleBoard/Elastic
         * 
         * @param drivetrain The pointer to the drivetrain subsystem
         * @param elevator The pointer to the elevator subsystem
         * @param intake The pointer to the intake subsystem
         * @param pivot The pointer to the pivot subsystem
         */
        AutonGenerator(
            DrivetrainSubsystem* drivetrain, 
            ElevatorSubsystem* elevator,
            IntakeSubsystem* intake, 
            PivotSubsystem* pivot
        );

        /**
         * Gets the autonomous commands based on the selected params in the dropdowns
         * 
         * @return The autonomous command group
         */
        frc2::CommandPtr GetAutonomousCommand();
        
    private:
        DrivetrainSubsystem* _drivetrain;
        ElevatorSubsystem* _elevator;
        IntakeSubsystem* _intake;
        PivotSubsystem* _pivot;

        frc::SendableChooser<Auton::Auton> _auton_chooser;
        frc::SendableChooser<AutonLevel::AutonLevel> _auton_level;
};

#endif
