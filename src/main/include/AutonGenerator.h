#ifndef AUTON_GENERATOR_H
#define AUTON_GENERATOR_H

// Subsystems
#include "subsystems/DrivetrainSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"
#include "subsystems/PivotSubsystem.h"
#include "subsystems/IntakeSubsystem.h"

//Auton Commands
#include "commands/auton/AutonBasicScoreCoralCommand.h"

#include <frc2/command/Commands.h>
#include <frc2/command/CommandPtr.h>
#include <frc/smartdashboard/SendableChooser.h>

class AutonGenerator {
    public:
        AutonGenerator(DrivetrainSubsystem* drivetrain, ElevatorSubsystem* elevator, PivotSubsystem* pivot, IntakeSubsystem* intake);
        frc2::CommandPtr GetAutonomousCommand(std::string_view type);
        
    private:
        std::unordered_map<std::string, frc2::CommandPtr> _auton_map;
        frc2::CommandPtr _GetCommand(std::string command_name);
        
        DrivetrainSubsystem* _drivetrain;
        ElevatorSubsystem* _elevator;
        PivotSubsystem* _pivot;
        IntakeSubsystem* _intake;

        frc::SendableChooser<std::string> _auton_chooser_initial;
        frc::SendableChooser<std::string> _auton_chooser_path_1;
        frc::SendableChooser<std::string> _auton_chooser_path_2;
        frc::SendableChooser<std::string> _auton_chooser_path_3;
};

#endif
