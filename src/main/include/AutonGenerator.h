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

#include <frc2/command/Commands.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc/smartdashboard/SendableChooser.h>

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
        /**
         * Takes a string of the name of the command and returns the command needed
         * 
         * @param command_name The name of the command to get
         * 
         * @return The command which can now be added to a command group
         */
        frc2::CommandPtr _GetCommand(std::string command_name);
        
        DrivetrainSubsystem* _drivetrain;
        ElevatorSubsystem* _elevator;
        IntakeSubsystem* _intake;
        PivotSubsystem* _pivot;

        std::vector<frc::SendableChooser<std::string>> _auton_choosers;
};

#endif
