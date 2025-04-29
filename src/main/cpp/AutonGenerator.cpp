#include "Constants.h"
#include "AutonGenerator.h"
#include "commands/auton/AutonStopCommand.h"

#include "commands/auton/AutonBasicScoreCoralCommand.h"

#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <memory>
#include <vector>

using namespace frc;
using namespace pathplanner;
using namespace SwerveConstants::AutonNames;

AutonGenerator::AutonGenerator(DrivetrainSubsystem* drivetrain, ElevatorSubsystem* elevator, IntakeSubsystem* intake, PivotSubsystem* pivot)
    : _drivetrain{drivetrain}, 
    _elevator{elevator}, 
    _intake{intake},
    _pivot{pivot} {

    _auton_chooser.SetDefaultOption("None", Auton::none);
    _auton_chooser.AddOption("Center", Auton::center);
    _auton_chooser.AddOption("Center Left", Auton::center_left);
    _auton_chooser.AddOption("Center Right", Auton::center_right);
    _auton_chooser.AddOption("Left", Auton::left);
    _auton_chooser.AddOption("Right", Auton::right);
    _auton_chooser.AddOption("Taxi", Auton::taxi);
    SmartDashboard::PutData("Auton", &_auton_chooser);

    // Adjust the level options to add more heights
    _auton_level.SetDefaultOption("None", AutonLevel::none);
    _auton_level.AddOption("Level 2", AutonLevel::level_2);
    _auton_level.AddOption("Level 4", AutonLevel::level_4);
    SmartDashboard::PutData("Auton Level", &_auton_level);
}

frc2::CommandPtr AutonGenerator::GetAutonomousCommand() {
    Auton::Auton auton_selected = _auton_chooser.GetSelected();
    AutonLevel::AutonLevel auton_level_selected = _auton_level.GetSelected();

    switch (auton_selected) {
        case Auton::center:
            // Go to G/H, score
            return frc2::cmd::Sequence(
                frc2::cmd::Sequence(
                    frc2::cmd::Defer([this]() { return _drivetrain->GoToPose(_drivetrain->GetReefSide("G", true)); }, {_drivetrain}),
                    frc2::cmd::Parallel(
                        frc2::cmd::Defer([this]() { return _drivetrain->GoToPose(_drivetrain->GetReefSide("G")); }, {_drivetrain}),
                        AutonScoreCoralCommand{_drivetrain, _elevator, _intake, _pivot, auton_level_selected}.ToPtr()
                    )
                ),
                AutonStopCommand(_drivetrain).ToPtr()
            );
            break;

        case Auton::center_left:
            // Go to G, then feeder station, then go to D, score, go to feeder station, score C
            return frc2::cmd::Sequence(
                frc2::cmd::Sequence(
                    frc2::cmd::Defer([this]() { return _drivetrain->GoToPose(_drivetrain->GetReefSide("G", true)); }, {_drivetrain}),
                    frc2::cmd::Parallel(
                        frc2::cmd::Defer([this]() { return _drivetrain->GoToPose(_drivetrain->GetReefSide("G")); }, {_drivetrain}),
                        AutonScoreCoralCommand{_drivetrain, _elevator, _intake, _pivot, auton_level_selected}.ToPtr()
                    )
                ),
                StowArmCommand{_pivot, _elevator}.ToPtr(),

                frc2::cmd::Defer([this]() { return _drivetrain->GoToPose(_drivetrain->GetReefAvoidPose(ReefAlignment::left), false, true); }, {_drivetrain}),

                frc2::cmd::Parallel(
                    frc2::cmd::Defer([this]() { return _drivetrain->GoToPose(_drivetrain->GetClosestFeederStation()); }, {_drivetrain}),
                    AutonFeederCoralCommand(_drivetrain, _elevator, _intake, _pivot).ToPtr()
                ),

                frc2::cmd::Sequence(
                    frc2::cmd::Defer([this]() { return _drivetrain->GoToPose(_drivetrain->GetReefSide("D", true)); }, {_drivetrain}),
                    frc2::cmd::Parallel(
                        frc2::cmd::Defer([this]() { return _drivetrain->GoToPose(_drivetrain->GetReefSide("D")); }, {_drivetrain}),
                        AutonScoreCoralCommand{_drivetrain, _elevator, _intake, _pivot, auton_level_selected}.ToPtr()
                    )
                ),
                StowArmCommand{_pivot, _elevator}.ToPtr(),

                frc2::cmd::Parallel(
                    frc2::cmd::Defer([this]() { return _drivetrain->GoToPose(_drivetrain->GetClosestFeederStation()); }, {_drivetrain}),
                    AutonFeederCoralCommand(_drivetrain, _elevator, _intake, _pivot).ToPtr()
                ),

                frc2::cmd::Sequence(
                    frc2::cmd::Defer([this]() { return _drivetrain->GoToPose(_drivetrain->GetReefSide("C", true)); }, {_drivetrain}),
                    frc2::cmd::Parallel(
                        frc2::cmd::Defer([this]() { return _drivetrain->GoToPose(_drivetrain->GetReefSide("C")); }, {_drivetrain}),
                        AutonScoreCoralCommand{_drivetrain, _elevator, _intake, _pivot, auton_level_selected}.ToPtr()
                    )
                ),
                StowArmCommand{_pivot, _elevator}.ToPtr()
            );
            break;

        case Auton::center_right:
            // Go to H, then feeder station, then go to K, score, go to feeder station, score at L
            return frc2::cmd::Sequence(
                frc2::cmd::Sequence(
                    frc2::cmd::Defer([this]() { return _drivetrain->GoToPose(_drivetrain->GetReefSide("H", true)); }, {_drivetrain}),
                    frc2::cmd::Parallel(
                        frc2::cmd::Defer([this]() { return _drivetrain->GoToPose(_drivetrain->GetReefSide("H")); }, {_drivetrain}),
                        AutonScoreCoralCommand{_drivetrain, _elevator, _intake, _pivot, auton_level_selected}.ToPtr()
                    )
                ),
                StowArmCommand{_pivot, _elevator}.ToPtr(),

                frc2::cmd::Defer([this]() { return _drivetrain->GoToPose(_drivetrain->GetReefAvoidPose(ReefAlignment::right), false, true); }, {_drivetrain}),

                frc2::cmd::Parallel(
                    frc2::cmd::Defer([this]() { return _drivetrain->GoToPose(_drivetrain->GetClosestFeederStation()); }, {_drivetrain}),
                    AutonFeederCoralCommand(_drivetrain, _elevator, _intake, _pivot).ToPtr()
                ),

                frc2::cmd::Sequence(
                    frc2::cmd::Defer([this]() { return _drivetrain->GoToPose(_drivetrain->GetReefSide("K", true)); }, {_drivetrain}),
                    frc2::cmd::Parallel(
                        frc2::cmd::Defer([this]() { return _drivetrain->GoToPose(_drivetrain->GetReefSide("K")); }, {_drivetrain}),
                        AutonScoreCoralCommand{_drivetrain, _elevator, _intake, _pivot, auton_level_selected}.ToPtr()
                    )
                ),
                StowArmCommand{_pivot, _elevator}.ToPtr(),

                frc2::cmd::Parallel(
                    frc2::cmd::Defer([this]() { return _drivetrain->GoToPose(_drivetrain->GetClosestFeederStation()); }, {_drivetrain}),
                    AutonFeederCoralCommand(_drivetrain, _elevator, _intake, _pivot).ToPtr()
                ),
                
                frc2::cmd::Sequence(
                    frc2::cmd::Defer([this]() { return _drivetrain->GoToPose(_drivetrain->GetReefSide("L", true)); }, {_drivetrain}),
                    frc2::cmd::Parallel(
                        frc2::cmd::Defer([this]() { return _drivetrain->GoToPose(_drivetrain->GetReefSide("L")); }, {_drivetrain}),
                        AutonScoreCoralCommand{_drivetrain, _elevator, _intake, _pivot, auton_level_selected}.ToPtr()
                    )
                ),
                StowArmCommand{_pivot, _elevator}.ToPtr()
            );
            break;

        case Auton::left:
            // Score E, then feeder station, then score D, feeder station, score C
            return frc2::cmd::Sequence(
                frc2::cmd::Sequence(
                    frc2::cmd::Defer([this]() { return _drivetrain->GoToPose(_drivetrain->GetReefSide("H", true)); }, {_drivetrain}),
                    frc2::cmd::Parallel(
                        frc2::cmd::Defer([this]() { return _drivetrain->GoToPose(_drivetrain->GetReefSide("H")); }, {_drivetrain}),
                        AutonScoreCoralCommand{_drivetrain, _elevator, _intake, _pivot, auton_level_selected}.ToPtr()
                    )
                ),
                StowArmCommand{_pivot, _elevator}.ToPtr(),

                frc2::cmd::Defer([this]() { return _drivetrain->GoToPose(_drivetrain->GetReefAvoidPose(ReefAlignment::left), false, true); }, {_drivetrain}),

                frc2::cmd::Parallel(
                    frc2::cmd::Defer([this]() { return _drivetrain->GoToPose(_drivetrain->GetClosestFeederStation()); }, {_drivetrain}),
                    AutonFeederCoralCommand(_drivetrain, _elevator, _intake, _pivot).ToPtr()
                ),

                frc2::cmd::Sequence(
                    frc2::cmd::Defer([this]() { return _drivetrain->GoToPose(_drivetrain->GetReefSide("D", true)); }, {_drivetrain}),
                    frc2::cmd::Parallel(
                        frc2::cmd::Defer([this]() { return _drivetrain->GoToPose(_drivetrain->GetReefSide("D")); }, {_drivetrain}),
                        AutonScoreCoralCommand{_drivetrain, _elevator, _intake, _pivot, auton_level_selected}.ToPtr()
                    )
                ),
                StowArmCommand{_pivot, _elevator}.ToPtr(),

                frc2::cmd::Parallel(
                    frc2::cmd::Defer([this]() { return _drivetrain->GoToPose(_drivetrain->GetClosestFeederStation()); }, {_drivetrain}),
                    AutonFeederCoralCommand(_drivetrain, _elevator, _intake, _pivot).ToPtr()
                ),
                
                frc2::cmd::Sequence(
                    frc2::cmd::Defer([this]() { return _drivetrain->GoToPose(_drivetrain->GetReefSide("C", true)); }, {_drivetrain}),
                    frc2::cmd::Parallel(
                        frc2::cmd::Defer([this]() { return _drivetrain->GoToPose(_drivetrain->GetReefSide("C")); }, {_drivetrain}),
                        AutonScoreCoralCommand{_drivetrain, _elevator, _intake, _pivot, auton_level_selected}.ToPtr()
                    )
                ),
                StowArmCommand{_pivot, _elevator}.ToPtr()
            );
            break;

        case Auton::right:
            // Score J, then feeder station, then score K, feeder station, score L
            return frc2::cmd::Sequence(
                frc2::cmd::Sequence(
                    frc2::cmd::Defer([this]() { return _drivetrain->GoToPose(_drivetrain->GetReefSide("J", true)); }, {_drivetrain}),
                    frc2::cmd::Parallel(
                        frc2::cmd::Defer([this]() { return _drivetrain->GoToPose(_drivetrain->GetReefSide("J")); }, {_drivetrain}),
                        AutonScoreCoralCommand{_drivetrain, _elevator, _intake, _pivot, auton_level_selected}.ToPtr()
                    )
                ),
                StowArmCommand{_pivot, _elevator}.ToPtr(),

                frc2::cmd::Defer([this]() { return _drivetrain->GoToPose(_drivetrain->GetReefAvoidPose(ReefAlignment::right), false, true); }, {_drivetrain}),

                frc2::cmd::Parallel(
                    frc2::cmd::Defer([this]() { return _drivetrain->GoToPose(_drivetrain->GetClosestFeederStation()); }, {_drivetrain}),
                    AutonFeederCoralCommand(_drivetrain, _elevator, _intake, _pivot).ToPtr()
                ),

                frc2::cmd::Sequence(
                    frc2::cmd::Defer([this]() { return _drivetrain->GoToPose(_drivetrain->GetReefSide("K", true)); }, {_drivetrain}),
                    frc2::cmd::Parallel(
                        frc2::cmd::Defer([this]() { return _drivetrain->GoToPose(_drivetrain->GetReefSide("K")); }, {_drivetrain}),
                        AutonScoreCoralCommand{_drivetrain, _elevator, _intake, _pivot, auton_level_selected}.ToPtr()
                    )
                ),
                StowArmCommand{_pivot, _elevator}.ToPtr(),

                frc2::cmd::Parallel(
                    frc2::cmd::Defer([this]() { return _drivetrain->GoToPose(_drivetrain->GetClosestFeederStation()); }, {_drivetrain}),
                    AutonFeederCoralCommand(_drivetrain, _elevator, _intake, _pivot).ToPtr()
                ),
                
                frc2::cmd::Sequence(
                    frc2::cmd::Defer([this]() { return _drivetrain->GoToPose(_drivetrain->GetReefSide("L", true)); }, {_drivetrain}),
                    frc2::cmd::Parallel(
                        frc2::cmd::Defer([this]() { return _drivetrain->GoToPose(_drivetrain->GetReefSide("L")); }, {_drivetrain}),
                        AutonScoreCoralCommand{_drivetrain, _elevator, _intake, _pivot, auton_level_selected}.ToPtr()
                    )
                ),
                StowArmCommand{_pivot, _elevator}.ToPtr()
            );
            break;

        case Auton::taxi:
            // Drives forwards past the starting line
            return frc2::cmd::Sequence(
                PathPlannerAuto("Drive").ToPtr(),
                AutonStopCommand(_drivetrain).ToPtr()
            );
            break;

        case Auton::none:
            // Doesn't do anything
            return frc2::cmd::None();
            break;
        default:
            return frc2::cmd::None();
    }
}
