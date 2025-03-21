#include "Constants.h"
#include "AutonGenerator.h"
#include "commands/auton/AutonStopCommand.h"

#include "commands/auton/AutonBasicScoreCoralCommand.h"

#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <memory>

using namespace frc;
using namespace pathplanner;
using namespace SwerveConstants::AutonNames;

AutonGenerator::AutonGenerator(DrivetrainSubsystem* drivetrain, ElevatorSubsystem* elevator, PivotSubsystem* pivot, IntakeSubsystem* intake)
    : _drivetrain{drivetrain}, _elevator{elevator}, _pivot{pivot}, _intake{intake} {
    
    _auton_chooser_initial.AddOption("No", "No");
    _auton_chooser_initial.SetDefaultOption("Yes", "Yes");

    _auton_chooser_path_1.SetDefaultOption("None","None");
    _auton_chooser_path_2.SetDefaultOption("None","None");
    _auton_chooser_path_3.SetDefaultOption("None","None");

    for (const std::string* ptr = std::begin(AUTON_NAMES); ptr != std::end(AUTON_NAMES); ptr++) {
        _auton_chooser_path_1.AddOption(*ptr, *ptr);
        _auton_chooser_path_2.AddOption(*ptr, *ptr);
        _auton_chooser_path_3.AddOption(*ptr, *ptr);
    }

    SmartDashboard::PutData("Score Initial Piece", &_auton_chooser_initial);
    SmartDashboard::PutData("Path 1", &_auton_chooser_path_1);
    SmartDashboard::PutData("Path 2", &_auton_chooser_path_2);
    SmartDashboard::PutData("Path 3", &_auton_chooser_path_3);
}

frc2::CommandPtr AutonGenerator::_GetCommand(std::string command_name) {
    if (command_name == "No" || command_name == "None") {
        return frc2::cmd::None();
    } else if (command_name == "Yes") {
        // Add command to perform the first scoring action
        return frc2::cmd::None();
    } else {
        return PathPlannerAuto(command_name).ToPtr();
    }
}

frc2::CommandPtr AutonGenerator::GetAutonomousCommand(std::string_view type) {
    //auto drive_path = PathPlannerAuto("Drive").ToPtr();

    if (type != "none") {
        _drivetrain->ResetOdometry(PathPlannerAuto("Drive").getStartingPose());
    }

    if (type == "drive") {
        return frc2::cmd::Sequence(
            PathPlannerAuto("Drive").ToPtr(),//&drive_path,
            AutonStopCommand(_drivetrain).ToPtr()
        );
    } else if (type == "score") {
        return frc2::cmd::Sequence(
            PathPlannerAuto("Drive").ToPtr(),//&drive_path,
            AutonStopCommand(_drivetrain).ToPtr(),
            AutonBasicScoreCoralCommand(_drivetrain, _elevator, _intake, _pivot).ToPtr()
        );
    } else {
        return frc2::cmd::None();
    }
    
    // return frc2::cmd::Sequence(
    //     _GetCommand(_auton_chooser_initial.GetSelected()),
    //     _GetCommand(_auton_chooser_path_1.GetSelected()),
    //     _GetCommand(_auton_chooser_path_2.GetSelected()),
    //     _GetCommand(_auton_chooser_path_3.GetSelected()),
    //     AutonStopCommand(_drivetrain).ToPtr()
    // );
}
