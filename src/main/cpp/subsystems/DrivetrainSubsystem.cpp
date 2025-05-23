#include "subsystems/DrivetrainSubsystem.h"

#include "commands/auton/FinalAlignmentCommand.h"
#include <frc/smartdashboard/SmartDashboard.h>

//Path Planner Paths
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>

using namespace SC;
using namespace SwerveConstants::DrivetrainConstants;
using namespace SwerveConstants::AutonDriveConstants;
using namespace VisionConstants;

using namespace frc;
using namespace units;
using namespace pathplanner;

DrivetrainSubsystem::DrivetrainSubsystem(SC_SwerveConfigs swerve_config_array[4], SC_Photon* vision, int pigeon_id, std::string_view drivetrain_canbus_name, Operator_Interface* oi, Driver_Interface* driver_interface)
        : _vision{vision}, _pigeon{pigeon_id, drivetrain_canbus_name}, _oi{oi}, _driver_interface{driver_interface}
{
    if (NULL != swerve_config_array) {
        wpi::array<frc::Rotation2d, 4> headings{wpi::empty_array};
        for (int i = 0; i < 4; i++) {
            if (FL == i || BL == i) {
                _modules[i] = new SwerveModule(swerve_config_array[i], DrivePIDConstants::LeftPID, drivetrain_canbus_name);
            } else {
                _modules[i] = new SwerveModule(swerve_config_array[i], DrivePIDConstants::RightPID, drivetrain_canbus_name);
            }
            headings[i] = _modules[i]->GetPosition().angle;
        }

        kinematics.ResetHeadings(headings);

        RobotConfig config = RobotConfig::fromGUISettings();

        AutoBuilder::configure(
        [this](){ return GetPose(); }, // Robot pose supplier
        [this](frc::Pose2d pose){ ResetOdometry(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this](){ return GetChassisSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](frc::ChassisSpeeds speeds){ DriveRobotcentric(speeds); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        std::make_shared<PPHolonomicDriveController>(
            PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
        ),
        config,
        []() {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                auto alliance = DriverStation::GetAlliance();
                if (alliance) {
                    return alliance.value() == DriverStation::Alliance::kRed;
                }
                return false;
            },
            this // Reference to this subsystem to set requirements
        );

        SmartDashboard::PutBoolean("Drivetrain Diagnostics", false);
    }

    _pigeon.GetConfigurator().Apply(ctre::phoenix6::configs::Pigeon2Configuration{});

    _odometry = new SwerveDrivePoseEstimator<4>{
                                                kinematics, 
                                                GetHeading(), 
                                                GetModulePositions(), 
                                                frc::Pose2d{}, 
                                                {0.1, 0.1, 0.1}, 
                                                {1.0, 1.0, 1.0}};
    SetBrakeMode();

    frc::SmartDashboard::PutData("Field", &_field);

    _alliance = frc::DriverStation::GetAlliance();
    if (!_alliance.has_value()) {
        fmt::print("Error: Teleop Drive Command failed to determine alliance station");
        _alliance = DriverStation::Alliance::kBlue;
    }
}

void DrivetrainSubsystem::Periodic() {
    if (_odometry == NULL) {
        fmt::print("Error: odometry accessed in Periodic before initialization");
    } else {
        _odometry->Update(GetHeading(), GetModulePositions());
        if (_vision != NULL && !_oi->IgnoreVision()){
            for (const SC::SC_CameraResults& results : _vision->GetCameraResults(GetPose())){
                wpi::array<double, 3> newStdDevs{results.Standard_Deviation(0), results.Standard_Deviation(1), results.Standard_Deviation(2)};
                _odometry->AddVisionMeasurement(results.Vision_Measurement, results.Timestamp, newStdDevs);
            }
        }
    }

    if (SmartDashboard::GetBoolean("Drivetrain Diagnostics", false)) {
        SmartDashboard::PutNumber("FL Encoder", _modules[FL]->GetPosition().angle.Degrees().value());
        SmartDashboard::PutNumber("FR Encoder", _modules[FR]->GetPosition().angle.Degrees().value());
        SmartDashboard::PutNumber("BL Encoder", _modules[BL]->GetPosition().angle.Degrees().value());
        SmartDashboard::PutNumber("BR Encoder", _modules[BR]->GetPosition().angle.Degrees().value());
        SmartDashboard::PutNumber("Gyro Heading", GetHeading().Degrees().value());
        SmartDashboard::PutNumber("Odometry X", GetPose().X().value());
        SmartDashboard::PutNumber("Odometry Y", GetPose().Y().value());
    }

    _field.SetRobotPose(GetPose());
    _field.GetObject("Target Position")->SetPose(_target_position);
}

void DrivetrainSubsystem::Drive(meters_per_second_t x_speed, meters_per_second_t y_speed, radians_per_second_t rotation, bool open_loop) {
    DriveRobotcentric(ChassisSpeeds::FromFieldRelativeSpeeds(x_speed, y_speed, rotation, GetHeading()), open_loop);
}

void DrivetrainSubsystem::DriveRobotcentric(ChassisSpeeds speeds, bool open_loop) {
    auto states = kinematics.ToSwerveModuleStates(speeds);
    SetModuleStates(states, open_loop, true);
}

void DrivetrainSubsystem::DynamicPivotDrive(meters_per_second_t x_speed, meters_per_second_t y_speed, radians_per_second_t rotation, frc::Translation2d center_of_rotation, bool open_loop) {
    DynamicPivotDriveRobotcentric(ChassisSpeeds::FromFieldRelativeSpeeds(x_speed, y_speed, rotation, GetHeading()), center_of_rotation, open_loop);
}

void DrivetrainSubsystem::DynamicPivotDriveRobotcentric(ChassisSpeeds speeds, frc::Translation2d center_of_rotation, bool open_loop) {
    auto states = kinematics.ToSwerveModuleStates(speeds, center_of_rotation);
    SetModuleStates(states, open_loop, true);
}


void DrivetrainSubsystem::SetModuleStates(wpi::array<SwerveModuleState, 4> desired_states, bool open_loop, bool optimize) {
    kinematics.DesaturateWheelSpeeds(&desired_states, MAX_WHEEL_SPEED);
    for (int i = 0; i < 4; i++) {
        if (_modules[i] == NULL) {
            fmt::print("Error: Swerve Module accessed in Periodic before initialization");
        } else {
            _modules[i]->SetDesiredState(desired_states[i], open_loop, optimize);
        }
    }
}

Rotation2d DrivetrainSubsystem::GetHeading() {
    return _pigeon.GetRotation2d().RotateBy(_pigeon_offset);
}

void DrivetrainSubsystem::SetHeading(degree_t heading) {
    ResetOdometry(Pose2d(_odometry->GetEstimatedPosition().Translation(), Rotation2d(heading)));
    //fmt::print("Reset Head!!!!!!\n");
}

degrees_per_second_t DrivetrainSubsystem::GetTurnRate() {
    return degrees_per_second_t{-_pigeon.GetAngularVelocityZWorld().GetValue()};
}

Pose2d DrivetrainSubsystem::GetPose() {
    if (_odometry == NULL) {
        fmt::print("Error: odometry accesed in GetPose before initialization");
        return Pose2d{0_m, 0_m, 0_deg};
    } else {
        return _odometry->GetEstimatedPosition();
    }
}

void DrivetrainSubsystem::ResetOdometry(Pose2d pose) {
    if (_odometry == NULL) {
        fmt::print("Error: odometry accesed in ResetOdometry before initialization");
        
    } else {
        _pigeon_offset = pose.Rotation().Degrees() - _pigeon.GetRotation2d().Degrees();

        _odometry->ResetPosition(GetHeading(), GetModulePositions(), pose);
    }
}

wpi::array<SwerveModulePosition, 4> DrivetrainSubsystem::GetModulePositions() {
    int checkNull = CheckNotNullModule();
    if (checkNull == 4) {
        return {_modules[FL]->GetPosition(), _modules[FR]->GetPosition(), _modules[BL]->GetPosition(), _modules[BR]->GetPosition()};
    } else {
        return {SwerveModulePosition(0_m, 0_deg), SwerveModulePosition(0_m, 0_deg), SwerveModulePosition(0_m, 0_deg), SwerveModulePosition(0_m, 0_deg)};
    }
}

ChassisSpeeds DrivetrainSubsystem::GetChassisSpeeds() {
    int checkNull = CheckNotNullModule();
    if (checkNull == 4) {
        return kinematics.ToChassisSpeeds({_modules[FL]->GetState(), _modules[FR]->GetState(), _modules[BL]->GetState(), _modules[BR]->GetState()});
    } else {
        return kinematics.ToChassisSpeeds({SwerveModuleState(0_mps, 0_deg), SwerveModuleState(0_mps, 0_deg), SwerveModuleState(0_mps, 0_deg), SwerveModuleState(0_mps, 0_deg)});
    }
}

void DrivetrainSubsystem::StopMotors() {
    if (_modules[FL] != NULL) {
        _modules[FL]->StopMotors();
    }
    if (_modules[FR] != NULL) {
        _modules[FR]->StopMotors();
    }
    if (_modules[BL] != NULL) {
        _modules[BL]->StopMotors();
    }
    if (_modules[BR] != NULL) {
        _modules[BR]->StopMotors();
    }
}

void DrivetrainSubsystem::ResetEncoders() {
    if (_modules[FL] != NULL) {
        _modules[FL]->ResetEncoder();
    }
    if (_modules[FR] != NULL) {
        _modules[FR]->ResetEncoder();
    }
    if (_modules[BL] != NULL) {
        _modules[BL]->ResetEncoder();
    }
    if (_modules[BR] != NULL) {
        _modules[BR]->ResetEncoder();
    }
    ResetOdometry(GetPose());
}

void DrivetrainSubsystem::SetCoastMode() {
    if (_modules[FL] != NULL) {
        _modules[FL]->SetCoastMode();
    }
    if (_modules[FR] != NULL) {
        _modules[FR]->SetCoastMode();
    }
    if (_modules[BL] != NULL) {
        _modules[BL]->SetCoastMode();
    }
    if (_modules[BR] != NULL) {
        _modules[BR]->SetCoastMode();
    }
}

void DrivetrainSubsystem::SetBrakeMode() {
    if (_modules[FL] != NULL) {
        _modules[FL]->SetBrakeMode();
    }
    if (_modules[FR] != NULL) {
        _modules[FR]->SetBrakeMode();
    }
    if (_modules[BL] != NULL) {
        _modules[BL]->SetBrakeMode();
    }
    if (_modules[BR] != NULL) {
        _modules[BR]->SetBrakeMode();
    }
}

int DrivetrainSubsystem::CheckNotNullModule() {
    int counter = 0;
    for (int i = 0; i < 4; i++) {
        if (_modules[i] != NULL) {
            counter++;
        }
    }
    return counter;
}

frc2::CommandPtr DrivetrainSubsystem::GoToPose(Pose2d pose, bool teleop, bool far) {
    _target_position = pose;

    if (teleop) {
        return frc2::cmd::Sequence(
            FinalAlignmentCommand{this, pose}.ToPtr(),
            this->RunOnce([this] {StopMotors();}),
            RumbleCommand(_driver_interface).ToPtr()
        );
    } else if (pose.Translation().Distance(GetPose().Translation()) > MINIMUM_PATHFIND_DISTANCE) {
        PathConstraints constraints = PathConstraints(MAX_LINEAR_SPEED, MAX_LINEAR_ACCELERATION, MAX_ROTATION_SPEED, MAX_ROTATION_ACCELERATION);
        std::vector<frc::Pose2d> poses{GetPose(), pose};

        std::vector<Waypoint> waypoints = PathPlannerPath::waypointsFromPoses(poses);

        ChassisSpeeds speed = GetChassisSpeeds();

        auto path = std::make_shared<PathPlannerPath>(
            waypoints,
            constraints,
            IdealStartingState(math::sqrt(speed.vx*speed.vx + speed.vy*speed.vy), math::atan2(speed.vy, speed.vx)), // The ideal starting state, this is only relevant for pre-planned paths, so can be nullopt for on-the-fly paths.
            GoalEndState(0.0_mps, pose.Rotation()) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        path->preventFlipping = true;
        fmt::print("Path Following");

        return frc2::cmd::Sequence(
            AutoBuilder::followPath(path), 
            FinalAlignmentCommand{this, pose, far}.ToPtr(),
            this->RunOnce([this] {StopMotors();})
        );
    } else {
        return frc2::cmd::Sequence(
            FinalAlignmentCommand{this, pose}.ToPtr(),
            this->RunOnce([this] {StopMotors();})
        );
    }
}

frc::Pose2d DrivetrainSubsystem::GetReefSide(std::string letter, bool far) {
    const auto& letter_to_id_map = (DriverStation::GetAlliance().value() == DriverStation::Alliance::kBlue) ? 
                                    APRIL_TAG_LETTER_TO_ID_BLUE : APRIL_TAG_LETTER_TO_ID_RED;
    auto it_id = letter_to_id_map.find(letter);

    if (it_id != letter_to_id_map.end()) {
        int id = it_id->second;
        
        for (const auto& tag : APRIL_TAG_LAYOUT.GetTags()) {
            if (tag.ID == id) {
                auto it_offset = APRIL_TAG_LETTER_TO_OFFSET.find(letter);
                if (it_offset != APRIL_TAG_LETTER_TO_OFFSET.end()) {
                    ReefAlignment::ReefAlignment reef_offset = it_offset->second;
                    if (far) {
                        if (reef_offset == ReefAlignment::left) {
                            return ApplyOffsetToPose(tag.pose.ToPose2d(), LEFT_REEF_OFFSET_FAR);
                        } else {
                            return ApplyOffsetToPose(tag.pose.ToPose2d(), RIGHT_REEF_OFFSET_FAR);
                        }
                    } else {
                        if (reef_offset == ReefAlignment::left) {
                            return ApplyOffsetToPose(tag.pose.ToPose2d(), LEFT_REEF_OFFSET);
                        } else {
                            return ApplyOffsetToPose(tag.pose.ToPose2d(), RIGHT_REEF_OFFSET);
                        }
                    }
                }
            }
        }
    }
    return frc::Pose2d{}; // Return a default pose if no match found
}

frc::Pose2d DrivetrainSubsystem::GetNearestPose(std::vector<frc::Pose2d> poses) {
    return GetPose().Nearest(std::span{poses});
}

frc::Pose2d DrivetrainSubsystem::ApplyOffsetToPose(frc::Pose2d pose, frc::Pose2d offset) {
    return frc::Pose2d{pose.Translation() + offset.Translation().RotateBy(pose.Rotation()), pose.Rotation() + offset.Rotation()};
}

frc::Pose2d DrivetrainSubsystem::GetClosestReefSide() {
    std::vector<Pose2d> poses;
    frc::Pose2d offset_pose;

    for (const auto& tag : APRIL_TAG_LAYOUT.GetTags()) {
        if (std::find(std::begin(REEF_APRIL_TAGS), std::end(REEF_APRIL_TAGS), tag.ID) != std::end(REEF_APRIL_TAGS)) {
            poses.emplace_back(ApplyOffsetToPose(tag.pose.ToPose2d(), LEFT_REEF_OFFSET));
            poses.emplace_back(ApplyOffsetToPose(tag.pose.ToPose2d(), RIGHT_REEF_OFFSET));
        }
    }

    return GetNearestPose(poses);
}

frc::Pose2d DrivetrainSubsystem::GetClosestFeederStation() {
    std::vector<Pose2d> poses;

    for (const auto& tag : APRIL_TAG_LAYOUT.GetTags()) {
        if (std::find(std::begin(FEEDER_STATION_APRIL_TAGS), std::end(FEEDER_STATION_APRIL_TAGS), tag.ID) != std::end(FEEDER_STATION_APRIL_TAGS)) {
            poses.emplace_back(ApplyOffsetToPose(tag.pose.ToPose2d(), LEFT_FEEDER_STATION_OFFSET));
            poses.emplace_back(ApplyOffsetToPose(tag.pose.ToPose2d(), RIGHT_FEEDER_STATION_OFFSET));
        }
    }

    return GetNearestPose(poses);
}

frc::Pose2d DrivetrainSubsystem::GetClosestProcessor() {
    std::vector<Pose2d> poses;

    for (const auto& tag : APRIL_TAG_LAYOUT.GetTags()) {
        if (std::find(std::begin(PROCESSOR_APRIL_TAGS), std::end(PROCESSOR_APRIL_TAGS), tag.ID) != std::end(PROCESSOR_APRIL_TAGS)) {
            poses.emplace_back(ApplyOffsetToPose(tag.pose.ToPose2d(), PROCESSOR_OFFSET));
        }
    }

    return GetNearestPose(poses);
}


frc::Pose2d DrivetrainSubsystem::GetReefAvoidPose(ReefAlignment::ReefAlignment alignment) {
    int tag_to_find = 99;

    if (_alliance == DriverStation::Alliance::kRed) {
        if (alignment == ReefAlignment::left) {
            tag_to_find = 4;
        } else if (alignment == ReefAlignment::right) {
            tag_to_find = 5;
        }
    } else {
        if (alignment == ReefAlignment::left) {
            tag_to_find = 15;
        } else if (alignment == ReefAlignment::right) {
            tag_to_find = 14;
        }
    }

    return ApplyOffsetToPose(APRIL_TAG_LAYOUT.GetTagPose(tag_to_find).value_or(frc::Pose3d{{GetPose().X() - 24_in, GetPose().Y(), GetPose().Rotation()}}).ToPose2d(), frc::Pose2d{BARGE_APRIL_TAG_OFFSET, frc::Rotation2d{GetPose().Rotation()}});
}

bool DrivetrainSubsystem::GetAtTargetPosition() {
    return GetPose().Translation().Distance(_target_position.Translation()) < AT_TARGET_POSITION_THRESHOLD;
}

bool DrivetrainSubsystem::GetNearTargetPosition() {
    return GetPose().Translation().Distance(_target_position.Translation()) < NEAR_TARGET_POSITION_THRESHOLD;
}