#ifndef VISION_H
#define VISION_H

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/apriltag/AprilTagFieldLayout.h>

#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>

class SC_Photon {
    public:
        SC_Photon(std::string_view camera_name, frc::AprilTagFieldLayout april_tag_layout, photon::PoseStrategy pose_strategy, frc::Transform3d camera_position);
        frc::Pose2d EstimatePose(frc::Pose2d current_pose);
    private:
        photon::PhotonCamera _camera;
        photon::PhotonPoseEstimator _pose_estimator;
};

#endif