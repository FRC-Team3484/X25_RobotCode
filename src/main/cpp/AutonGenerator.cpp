#include "Constants.h"
#include "AutonGenerator.h"
#include "commands/auton/AutonStopCommand.h"

#include "commands/auton/AutonBasicScoreCoralCommand.h"

#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <memory>
#include <vector>

#include <frc2/command/ProxyCommand.h>

using namespace frc;
using namespace pathplanner;
using namespace SwerveConstants::AutonNames;
using namespace SwerveConstants::AutonDropdowns;

AutonGenerator::AutonGenerator(DrivetrainSubsystem* drivetrain, ElevatorSubsystem* elevator, IntakeSubsystem* intake, PivotSubsystem* pivot)
    : _drivetrain{drivetrain}, 
    _elevator{elevator}, 
    _intake{intake},
    _pivot{pivot} {
    
    _auton_choosers.resize(AUTON_DROPDOWN_COUNT * 2);

    for (int i = 0; i < AUTON_DROPDOWN_COUNT * 2; i += 2) {
        _auton_choosers[i].SetDefaultOption("None", "None");
        _auton_choosers[i + 1].SetDefaultOption("None", "None");

        _auton_choosers[i].AddOption("A", "A");
        _auton_choosers[i].AddOption("B", "B");
        _auton_choosers[i].AddOption("C", "C");
        _auton_choosers[i].AddOption("D", "D");
        _auton_choosers[i].AddOption("E", "E");
        _auton_choosers[i].AddOption("F", "F");
        _auton_choosers[i].AddOption("G", "G");
        _auton_choosers[i].AddOption("H", "H");
        _auton_choosers[i].AddOption("I", "I");
        _auton_choosers[i].AddOption("J", "J");
        _auton_choosers[i].AddOption("K", "K");
        _auton_choosers[i].AddOption("L", "L");
        
        _auton_choosers[i + 1].AddOption("Level 1", "Level 1");
        _auton_choosers[i + 1].AddOption("Level 2", "Level 2");
        _auton_choosers[i + 1].AddOption("Level 3", "Level 3");
        _auton_choosers[i + 1].AddOption("Level 4", "Level 4");
    }

    for (int i = 0; i < AUTON_DROPDOWN_COUNT * 2; i += 2) {
        SmartDashboard::PutData("Auton " + std::to_string(i / 2 + 1) + " Score Alignment", &_auton_choosers[i]);
        SmartDashboard::PutData("Auton " + std::to_string(i / 2 + 1) + " Score", &_auton_choosers[i + 1]);
    }
}

frc2::CommandPtr AutonGenerator::_GetCommand(std::string command_name) {

    std::vector<frc2::CommandPtr> commands;

    if (command_name == "None") {
        return frc2::cmd::None();

    } else if (command_name == "A" || command_name == "B" || command_name == "C" || command_name == "D" || command_name == "E" || command_name == "F" || command_name == "G" || command_name == "H" || command_name == "I" || command_name == "J" || command_name == "K" || command_name == "L") {
        commands.push_back(frc2::cmd::DeferredProxy([this, command_name]() { return _drivetrain->GoToPose(_drivetrain->GetReefSide(command_name)); } ));
        // commands.push_back(frc2::cmd::None());

    } else if (command_name == "Level 1" || command_name == "Level 2" || command_name == "Level 3" || command_name == "Level 4") {
        commands.push_back(AutonScoreCoralCommand{_drivetrain, _elevator, _intake, _pivot, command_name}.ToPtr());

    } else {
        return frc2::cmd::None();
    }

    return frc2::cmd::Sequence(std::move(commands));
}

frc2::CommandPtr AutonGenerator::GetAutonomousCommand() {
    std::vector<frc2::CommandPtr> commands;
    for (int i = 0; i < AUTON_DROPDOWN_COUNT; i++) {
        fmt::println("Loop");
        commands.push_back(_GetCommand(_auton_choosers[i].GetSelected()));
        
        if (i % 2 == 1 && i < AUTON_DROPDOWN_COUNT - 1) {
            commands.push_back(AutonFeederCoralCommand{_drivetrain, _elevator, _intake, _pivot}.ToPtr());
        }
    }
    commands.push_back(AutonStopCommand(_drivetrain).ToPtr());

    fmt::println("Made command list");
    
    return frc2::cmd::Sequence(std::move(commands));
}
