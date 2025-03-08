#include "Constants.h"
#include "AutonGenerator.h"
#include "commands/auton/AutonStopCommand.h"

#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <memory>
#include <vector>

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

        _auton_choosers[i].AddOption(AUTON_SCORE_ALIGNMENT[1], "A");
        _auton_choosers[i].AddOption(AUTON_SCORE_ALIGNMENT[2], "B");
        _auton_choosers[i].AddOption(AUTON_SCORE_ALIGNMENT[3], "C");
        _auton_choosers[i].AddOption(AUTON_SCORE_ALIGNMENT[4], "D");
        _auton_choosers[i].AddOption(AUTON_SCORE_ALIGNMENT[5], "E");
        _auton_choosers[i].AddOption(AUTON_SCORE_ALIGNMENT[6], "F");
        _auton_choosers[i].AddOption(AUTON_SCORE_ALIGNMENT[7], "G");
        _auton_choosers[i].AddOption(AUTON_SCORE_ALIGNMENT[8], "H");
        _auton_choosers[i].AddOption(AUTON_SCORE_ALIGNMENT[9], "I");
        _auton_choosers[i].AddOption(AUTON_SCORE_ALIGNMENT[10], "J");
        _auton_choosers[i].AddOption(AUTON_SCORE_ALIGNMENT[11], "K");
        _auton_choosers[i].AddOption(AUTON_SCORE_ALIGNMENT[12], "L");
        
        _auton_choosers[i + 1].AddOption(AUTON_SCORE_LEVEL[1], "Level 1");
        _auton_choosers[i + 1].AddOption(AUTON_SCORE_LEVEL[2], "Level 2");
        _auton_choosers[i + 1].AddOption(AUTON_SCORE_LEVEL[3], "Level 3");
        _auton_choosers[i + 1].AddOption(AUTON_SCORE_LEVEL[4], "Level 4");
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
        commands.push_back(_GetCommand(_auton_choosers[i].GetSelected()));
        
        if (i % 2 == 1 && i < AUTON_DROPDOWN_COUNT - 1) {
            commands.push_back(AutonFeederCoralCommand{_drivetrain, _elevator, _intake, _pivot}.ToPtr());
        }
    }
    commands.push_back(AutonStopCommand(_drivetrain).ToPtr());
    
    return frc2::cmd::Sequence(std::move(commands));
}
