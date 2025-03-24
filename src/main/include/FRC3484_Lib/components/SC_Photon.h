#ifndef VISION_H
#define VISION_H

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/apriltag/AprilTagFieldLayout.h>

#include <FRC3484_Lib/utils/SC_Datatypes.h>

#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <photon/PhotonUtils.h>

class SC_Photon {
    public:
        SC_Photon(std::vector<SC::SC_CameraConfig> camera_configs, frc::AprilTagFieldLayout april_tag_layout, photon::PoseStrategy pose_strategy);
        frc::Pose2d EstimatePose(frc::Pose2d current_pose);
        frc::Pose2d AveragePoses(std::vector<frc::Pose2d> poses);
        frc::Pose2d ConvertPose(frc::Pose3d pose);
    private:
        std::vector<photon::PhotonCamera*> _cameras;
        std::vector<photon::PhotonPoseEstimator*> _pose_estimators;
};

#endif