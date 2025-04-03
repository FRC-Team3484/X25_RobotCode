#include "FRC3484_Lib/components/SC_Photon.h"

using namespace frc;
using namespace photon;
using namespace std;

SC_Photon::SC_Photon(std::vector<SC::SC_CameraConfig> camera_configs, frc::AprilTagFieldLayout april_tag_layout, photon::PoseStrategy pose_strategy) {
    for (size_t i = 0; i < camera_configs.size(); i++) {
        if (camera_configs[i].Camera_Enabled){
            _cameras.emplace_back(new PhotonCamera(camera_configs[i].Camera_Name));
            _pose_estimators.emplace_back(new PhotonPoseEstimator(april_tag_layout, pose_strategy, camera_configs[i].Camera_Position));
        }
    }
}

std::vector<SC::SC_CameraResults> SC_Photon::GetCameraResults(frc::Pose2d current_pose){
    vector<SC::SC_CameraResults> CamResults;

    for (size_t i = 0; i < _pose_estimators.size(); i++) {
        if (_pose_estimators[i]->GetPoseStrategy() == PoseStrategy::CLOSEST_TO_REFERENCE_POSE) {
            _pose_estimators[i]->SetReferencePose(Pose3d{current_pose});
        }

        std::vector<PhotonPipelineResult> results = _cameras[i]->GetAllUnreadResults();
        std::optional<EstimatedRobotPose> vision_est;
        for (const PhotonPipelineResult& result: results) {
            vision_est = _pose_estimators[i]->Update(result);
        }
        if (vision_est.has_value()){
            EstimatedRobotPose est = vision_est.value();
            CamResults.emplace_back(
                SC::SC_CameraResults{
                    est.estimatedPose.ToPose2d(), 
                    est.timestamp,
                    GetEstimatedStdDevs(
                                        results.back(), 
                                        est.estimatedPose.ToPose2d(), 
                                        *_pose_estimators[_pose_estimators.size()-1]
                                        )
                }
            );
        }
    }
    return CamResults;
}

Eigen::Matrix<double, 3, 1> SC_Photon::GetEstimatedStdDevs(photon::PhotonPipelineResult result, frc::Pose2d pose,  photon::PhotonPoseEstimator photonEstimator){
    Eigen::Matrix<double, 3, 1> estStdDev = 
        VisionConstants::SINGLE_TAG_STDDEV;
    std::span<const photon::PhotonTrackedTarget> targets = result.GetTargets();
    int numTags = 0;
    units::meter_t averageDistance = 0_m;
    for (const photon::PhotonTrackedTarget& tgt : targets) {
        std::optional<frc::Pose3d> tagPose = photonEstimator.GetFieldLayout().GetTagPose(tgt.GetFiducialId());
        if (tagPose){
            numTags++;
            averageDistance += tagPose->ToPose2d().Translation().Distance(pose.Translation());
        }
    }
    if (numTags == 0) return estStdDev;
    averageDistance /= numTags;
    if (numTags > 1) estStdDev = VisionConstants::MULTI_TAG_STDDEV;
    if (numTags == 1 && averageDistance > 4_m){
        estStdDev = (Eigen::MatrixXd(3, 1) << std::numeric_limits<double>::max(),
                     std::numeric_limits<double>::max(),
                     std::numeric_limits<double>::max()).finished();
    } else estStdDev = estStdDev * (1+ (averageDistance.value() * averageDistance.value() / 30));
    return estStdDev;
}