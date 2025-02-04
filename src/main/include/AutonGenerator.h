#ifndef AUTON_GENERATOR_H
#define AUTON_GENERATOR_H

// Subsystems
#include "subsystems/DrivetrainSubsystem.h"
// Include other subsystems here

//Auton Commands
// Include auton commands here

#include <frc2/command/Commands.h>
#include <frc2/command/CommandPtr.h>
#include <frc/smartdashboard/SendableChooser.h>

class AutonGenerator {
    public:
        AutonGenerator(DrivetrainSubsystem* drivetrain);
        frc2::CommandPtr GetAutonomousCommand();
        
    private:
        std::unordered_map<std::string, frc2::CommandPtr> _auton_map;
        frc2::CommandPtr _GetCommand(std::string command_name);
        
        DrivetrainSubsystem* _drivetrain;

        frc::SendableChooser<std::string> _auton_chooser_initial;
        frc::SendableChooser<std::string> _auton_chooser_path_1;
        frc::SendableChooser<std::string> _auton_chooser_path_2;
        frc::SendableChooser<std::string> _auton_chooser_path_3;
};

#endif