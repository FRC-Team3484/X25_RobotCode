// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "FRC3484_Lib/components/SC_Photon.h"

using namespace frc;
using namespace photon;

SC_Photon::SC_Photon(std::string_view camera_name, AprilTagFieldLayout april_tag_layout, photon::PoseStrategy pose_strategy, Transform3d camera_position)
        : _camera(camera_name), _pose_estimator(april_tag_layout, pose_strategy, camera_position) {}

Pose2d SC_Photon::EstimatePose(Pose2d current_pose) {
    if (_pose_estimator.GetPoseStrategy() == PoseStrategy::CLOSEST_TO_REFERENCE_POSE)
        _pose_estimator.SetReferencePose(Pose3d{current_pose});

    PhotonPipelineResult&& result = _camera.GetLatestResult();
    std::optional<EstimatedRobotPose> vision_est = _pose_estimator.Update(result);
    if (vision_est.has_value()) return vision_est->estimatedPose.ToPose2d();
    return current_pose;
}