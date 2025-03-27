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
	return _counter > FINAL_ALIGN_EXIT || 
	(_drivetrain_subsystem->GetPose().Translation().Distance(_target_pose.Translation()) < FINAL_POSE_TOLERANCE && 
	abs(_drivetrain_subsystem->GetPose().Rotation().Degrees() - _target_pose.Rotation().Degrees()) < FINAL_ANGLE_TOLERANCE);
}
