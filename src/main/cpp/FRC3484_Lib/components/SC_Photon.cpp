#include "FRC3484_Lib/components/SC_Photon.h"

using namespace frc;
using namespace photon;

SC_Photon::SC_Photon(std::vector<SC::SC_CameraConfig> camera_configs, frc::AprilTagFieldLayout april_tag_layout, photon::PoseStrategy pose_strategy) {
    for (int i = 0; i < camera_configs.size(); i++) {
        _cameras.emplace_back(new PhotonCamera(camera_configs[i].Camera_Name));
        _pose_estimators.emplace_back(new PhotonPoseEstimator(april_tag_layout, pose_strategy, camera_configs[i].Camera_Position));
    }
}

Pose2d SC_Photon::EstimatePose(Pose2d current_pose) {
    std::vector<Pose2d> poses;

    for (int i = 0; i < _pose_estimators.size(); i++) {
        if (_pose_estimators[i]->GetPoseStrategy() == PoseStrategy::CLOSEST_TO_REFERENCE_POSE) {
            _pose_estimators[i]->SetReferencePose(Pose3d{current_pose});
        }

        std::vector<PhotonPipelineResult> results = _cameras[i]->GetAllUnreadResults();
        std::optional<EstimatedRobotPose> vision_est;
        for (const PhotonPipelineResult& result: results) {
            vision_est = _pose_estimators[i]->Update(result);
        }

        if (vision_est.has_value()) {
            poses.emplace_back(vision_est->estimatedPose.ToPose2d());
        } else {
            return current_pose;
        }
    }
    
    return AveragePoses(poses);
}

Pose2d SC_Photon::AveragePoses(std::vector<frc::Pose2d> poses) {
    if (poses.empty()) {
        return Pose2d{0_m, 0_m, 0_deg};
    }

    units::meter_t x = 0_m;
    units::meter_t y = 0_m;
    double sin_sum = 0.0;
    double cos_sum = 0.0;

    for (const Pose2d& pose : poses) {
        x += pose.Translation().X();
        y += pose.Translation().Y();
        sin_sum += pose.Rotation().Sin();
        cos_sum += pose.Rotation().Cos();
    }

    int count = poses.size();
    return Pose2d{(x / count), (y / count), Rotation2d(sin_sum, cos_sum)};
}
