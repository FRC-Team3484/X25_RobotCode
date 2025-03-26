#include "commands/auton/FinalAlignmentCommand.h"
#include "Constants.h"

FinalAlignmentCommand::FinalAlignmentCommand(
    DrivetrainSubsystem* drivetrain_subsystem,
    frc::Pose2d target_pose) :
    _drivetrain_subsystem(drivetrain_subsystem){
    AddRequirements(_drivetrain_subsystem);
}

void FinalAlignmentCommand::Initialize() {}

void FinalAlignmentCommand::Execute() {
	_counter++;
	pathplanner::PathPlannerTrajectoryState goalSate{};

	_drivetrain_subsystem->DriveRobotcentric(
		_drive_controller->calculateRobotRelativeSpeeds(
			_drivetrain_subsystem->GetPose(), 
			goalSate
		)
	);

}

void FinalAlignmentCommand::End(bool interrupted) {
}

bool FinalAlignmentCommand::IsFinished() {
	return _counter > 35;
}
