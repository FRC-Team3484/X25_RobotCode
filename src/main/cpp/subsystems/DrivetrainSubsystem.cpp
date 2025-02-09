// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DrivetrainSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>

//Path Planner Paths
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <frc/DriverStation.h>

using namespace SC;
using namespace SwerveConstants::DrivetrainConstants;
using namespace SwerveConstants::AutonDriveConstants;
using namespace VisionConstants;

using namespace frc;
using namespace units;
using namespace pathplanner;

DrivetrainSubsystem::DrivetrainSubsystem(SC_SwerveConfigs swerve_config_array[4], SC_Photon* vision)
        : _vision{vision}
{
    if (NULL != swerve_config_array) {
        wpi::array<frc::Rotation2d, 4> headings{wpi::empty_array};
        for (int i = 0; i < 4; i++) {
            if (FL == i || BL == i) {
                _modules[i] = new SwerveModule(swerve_config_array[i], DrivePIDConstants::LeftPID);
            } else {
                _modules[i] = new SwerveModule(swerve_config_array[i], DrivePIDConstants::RightPID);
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

    _gyro = new studica::AHRS{studica::AHRS::NavXComType::kMXP_SPI};
    _odometry = new SwerveDriveOdometry<4>{kinematics, GetHeading(), GetModulePositions()};
    SetBrakeMode();

    frc::SmartDashboard::PutData("Field", &_field);
}

void DrivetrainSubsystem::Periodic() {
    if (_odometry == NULL) {
        fmt::print("Error: odometry accessed in Periodic before initialization");
    } else {
        _odometry->Update(GetHeading(), GetModulePositions());
        if (_vision != NULL)
            ResetOdometry(_vision->EstimatePose(GetPose()));
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
}

void DrivetrainSubsystem::Drive(meters_per_second_t x_speed, meters_per_second_t y_speed, radians_per_second_t rotation, bool open_loop) {
    DriveRobotcentric(ChassisSpeeds::FromFieldRelativeSpeeds(x_speed, y_speed, rotation, GetHeading()), open_loop);
}

void DrivetrainSubsystem::DriveRobotcentric(ChassisSpeeds speeds, bool open_loop) {
    auto states = kinematics.ToSwerveModuleStates(speeds);
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
    if (_gyro == NULL) {
        fmt::print("Error: gyro accessed in GetHeading before initialization");
        return Rotation2d{0_deg};

    } else {
        return degree_t{-_gyro->GetAngle()} + _gyro_offset;
    }
}

void DrivetrainSubsystem::SetHeading(degree_t heading) {
    ResetOdometry(Pose2d(_odometry->GetPose().Translation(), Rotation2d(heading)));
    //fmt::print("Reset Head!!!!!!\n");
}

degrees_per_second_t DrivetrainSubsystem::GetTurnRate() {
    if (_gyro == NULL) {
        fmt::print("Error: gyro accessed in GetTurnRate before initialization");
        return 0_deg_per_s;
    } else {
        return degrees_per_second_t{_gyro->GetRate()};
    }
}

Pose2d DrivetrainSubsystem::GetPose() {
    if (_odometry == NULL) {
        fmt::print("Error: odometry accesed in GetPose before initialization");
        return Pose2d{0_m, 0_m, 0_deg};
    } else {
        return _odometry->GetPose();
    }
}

void DrivetrainSubsystem::ResetOdometry(Pose2d pose) {
    if (_odometry == NULL) {
        fmt::print("Error: odometry accesed in ResetOdometry before initialization");

    } else if (_gyro == NULL) {
        fmt::print("Error: gyro accessed in ZeroHeading before initialization");
        
    } else {
        //fmt::print("Resetting Pose: X ({0}), Y({1}), Rotation({2})", pose.X().value(), pose.Y().value(), pose.Rotation().Degrees().value());
        _gyro_offset = pose.Rotation().Degrees() + degree_t{_gyro->GetAngle()};
        //_gyro->ZeroYaw();
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

frc2::CommandPtr DrivetrainSubsystem::GoToPose(Pose2d pose) {
    PathConstraints constraints = PathConstraints(MAX_LINEAR_SPEED, MAX_LINEAR_ACCELERATION, MAX_ROTATION_SPEED, MAX_ROTATION_ACCELERATION);

    frc2::CommandPtr pathfindingCommand = AutoBuilder::pathfindToPose(
        pose,
        constraints,
        0.0_mps
    );

    return pathfindingCommand;
}

frc::Pose2d DrivetrainSubsystem::GetClosestReefSide(SwerveConstants::AutonDriveConstants::REEF_OFFSETS reef_offset) {
    frc::Pose2d current_pose = GetPose();
    std::vector<Pose2d> differences;
    frc::Pose2d offset_pose;

    for (const auto& tag : APRIL_TAG_LAYOUT.GetTags()) {
        if (std::find(std::begin(REEF_APRIL_TAGS), std::end(REEF_APRIL_TAGS), tag.ID) != std::end(REEF_APRIL_TAGS)) {
            differences.emplace_back(Pose2d{tag.pose.ToPose2d().Translation() - current_pose.Translation(), tag.pose.ToPose2d().Rotation() - current_pose.Rotation()});
        }
    }

    frc::Pose2d closest = frc::Pose2d().Nearest(std::span{differences});
    frc::Pose2d april_tag = Pose2d{current_pose.Translation() + closest.Translation(), current_pose.Rotation() + closest.Rotation()};

    if (reef_offset == REEF_OFFSETS::left) {
        offset_pose = Pose2d{april_tag.Translation() + LEFT_REEF_OFFSET, april_tag.Rotation()};
    } else if (reef_offset == REEF_OFFSETS::right) {
        offset_pose = Pose2d{april_tag.Translation() + RIGHT_REEF_OFFSET, april_tag.Rotation()};
    }

    return offset_pose;
}
