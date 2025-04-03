#include "commands/auton/FinalAlignmentCommand.h"
#include <units/math.h>
#include "Constants.h"

using namespace SwerveConstants::DrivetrainConstants;
using namespace units::math;

FinalAlignmentCommand::FinalAlignmentCommand(
    DrivetrainSubsystem* drivetrain_subsystem,
    frc::Pose2d target_pose) :
    _drivetrain_subsystem(drivetrain_subsystem){
    AddRequirements(_drivetrain_subsystem);
	_target_pose = target_pose;
}

void FinalAlignmentCommand::Initialize() {
	_counter = 0;
}

void FinalAlignmentCommand::Execute() {
	_counter++;
	pathplanner::PathPlannerTrajectoryState goalState{};
	goalState.pose = _target_pose;

	_drivetrain_subsystem->DriveRobotcentric(
		_drive_controller->calculateRobotRelativeSpeeds(
			_drivetrain_subsystem->GetPose(), 
			goalState
		)
	);
}

void FinalAlignmentCommand::End(bool interrupted) {
	_drivetrain_subsystem->StopMotors();
}

bool FinalAlignmentCommand::IsFinished() {
	return _counter > FINAL_ALIGN_EXIT || 
	(_drivetrain_subsystem->GetPose().Translation().Distance(_target_pose.Translation()) < FINAL_POSE_TOLERANCE && 
	abs(_drivetrain_subsystem->GetPose().Rotation().Degrees() - _target_pose.Rotation().Degrees()) < FINAL_ANGLE_TOLERANCE);
}
