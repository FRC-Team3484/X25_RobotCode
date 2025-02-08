#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <photon/PhotonPoseEstimator.h>

#include <FRC3484_Lib/utils/SC_ControllerMaps.h>
#include <FRC3484_Lib/utils/SC_Datatypes.h>

#include <units/length.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/voltage.h>
#include <units/time.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>

namespace VisionConstants {
    const frc::AprilTagFieldLayout APRIL_TAG_LAYOUT = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2025Reefscape);
    constexpr photon::PoseStrategy POSE_STRATEGY = photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR;

    constexpr std::string_view CAMERA_NAME = "Camera_1";
    const frc::Transform3d CAMERA_POSITION = frc::Transform3d{frc::Translation3d{12_in, 0_in, 12_in}, frc::Rotation3d{0_deg, -30_deg, 0_deg}};
}

namespace SwerveConstants {
    namespace AutonNames {
        const std::string AUTON_NAMES[] = {
            "Path1", "Path2", "Path3"
        };
    }

    namespace ControllerConstants {
        constexpr double DRIVER_RUMBLE_HIGH = 0.5;
        constexpr double DRIVER_RUMBLE_LOW = 0.2;

        constexpr double OPERATOR_RUMBLE_HIGH = 0.5;
        constexpr double OPERATOR_RUMBLE_LOW = 0.2;
        
        constexpr double RUMBLE_STOP = 0;
    }

    namespace DrivetrainConstants {
        // Swerve Module Configurations

        // For those with static, do not change into constants; it will break the linking
        // DO NOT REMOVE STATIC CALLS

        // Drive, steer, encoder, magnet offset
        static SC::SC_SwerveConfigs SWERVE_FRONT_LEFT{10,11,20, -90.527_deg}; //-92.505
        static SC::SC_SwerveConfigs SWERVE_FRONT_RIGHT{12,13,21, -58.096_deg}; //-60.205
        static SC::SC_SwerveConfigs SWERVE_BACK_LEFT{14,15,22, 160.225_deg}; //160.654
        static SC::SC_SwerveConfigs SWERVE_BACK_RIGHT{16,17,23, -55.898_deg}; //-55.283

        static SC::SC_SwerveConfigs SWERVE_CONFIGS_ARRAY[4] = {
            SWERVE_FRONT_LEFT,
            SWERVE_FRONT_RIGHT,
            SWERVE_BACK_LEFT,
            SWERVE_BACK_RIGHT
        };

        #define FL 0
        #define FR 1
        #define BL 2
        #define BR 3

        constexpr units::inch_t DRIVETRAIN_WIDTH = 24_in;
        constexpr units::inch_t DRIVETRAIN_LENGTH = 24_in;
        constexpr double DRIVE_GEAR_RATIO = 36000.0/5880.0;
        constexpr double STEER_GEAR_RATIO = 12.8;
        constexpr units::inch_t WHEEL_RADIUS = 2_in;

        constexpr units::feet_per_second_t MAX_WHEEL_SPEED = 8_fps;
        constexpr units::feet_per_second_squared_t MAX_WHEEL_ACCELERATION = 4_fps_sq;

// Check For Autons
        namespace DrivePIDConstants {
            // Check SC_Datatypes for the struct
            [[maybe_unused]] static SC::SC_SwervePID LeftPID{2, 0, 0, 2.0715 * 1_V / 1_mps, 0.17977 * 1_V / 1_mps_sq, 0.77607_V};
            [[maybe_unused]] static SC::SC_SwervePID RightPID{2, 0, 0, 2.0802 * 1_V / 1_mps, 0.30693 * 1_V / 1_mps_sq, 0.73235_V};
        }
        namespace SteerPIDConstants {
            constexpr double Kp_Steer = 0.5;
            constexpr double Ki_Steer = 0.0;
            constexpr double Kd_Steer = 0.0;
            constexpr units::radians_per_second_t MAX_SPEED = 12_rad_per_s;
            constexpr units::radians_per_second_squared_t MAX_ACCELERATION = 100_rad_per_s_sq;
        }

        namespace JoystickScaling {
            constexpr double LOW_SCALE = 0.35;
        }
    }

    namespace BrakeConstants {
        constexpr auto DYNAMIC_BRAKE_SCALING = -0.02/1_in;
        constexpr units::second_t BRAKE_DELAY = .5_s;
    }

    namespace AutonDriveConstants {
        // How fast the robot can move in autons
        constexpr units::feet_per_second_t MAX_LINEAR_SPEED = 8_fps;
        constexpr units::feet_per_second_squared_t MAX_LINEAR_ACCELERATION = 4_fps_sq;
        constexpr units::radians_per_second_t MAX_ROTATION_SPEED = 5.431_rad_per_s;
        constexpr units::radians_per_second_squared_t MAX_ROTATION_ACCELERATION = 2_rad_per_s_sq;

        constexpr units::inch_t POSITION_TOLERANCE = 2_in; // Drive to a position, when safe to quit
        constexpr units::degree_t ANGLE_TOLERANCE = 2_deg;
    }

    namespace PathDrivePIDConstants {
        constexpr double P = 5.0;
        constexpr double I = 0.0;
        constexpr double D = 0.0;
    }

    namespace PathRotationPIDConstants {
        constexpr double P = 2.0;
        constexpr double I = 0.0;
        constexpr double D = 0.0;
    }
}

namespace UserInterface {
    namespace Driver {
        constexpr int DRIVER_CONTROLLER_PORT = 0;
        constexpr double DRIVER_JOYSTICK_DEADBAND = 0.02;
        // Motion
        constexpr int THROTTLE = XBOX_LS_Y;
        constexpr int STRAFE = XBOX_LS_X;
        constexpr int ROTATION = XBOX_RS_X;
        // Settings
        constexpr int RESET_HEADING = XBOX_BACK;
        constexpr int BRAKE = XBOX_X;
        constexpr int BRAKE_MODE = XBOX_RB;
        constexpr int DISABLE_BRAKE_MODE = XBOX_LB;
        constexpr int LOW_SPEED = XBOX_LT;

        // Override
        constexpr int DRIVER_OVERRIDE = XBOX_Y;

    }
    namespace Operator {
        constexpr int OPERATOR_CONTROLLER_PORT = 1;

    }
    namespace Testing {
        constexpr int TESTING_OPEN_LOOP_LEFT = XBOX_LS_Y;
        constexpr int TESTING_OPEN_LOOP_RIGHT = XBOX_RS_Y;
        constexpr int TESTING_GET_MOTOR = XBOX_LS_Y;
        constexpr double TESTING_DEADBAND = 0.02;
        constexpr int TESTING_CONTROLLER_PORT = 2;
    }
}

namespace ElevatorConstants {
    constexpr int PRIMARY_MOTOR_CAN_ID = 30;
    constexpr int SECONDARY_MOTOR_CAN_ID = 31;
    constexpr int HOME_SENSOR_DI_CH = 0;

    constexpr units::feet_per_second_t MAX_VELOCITY = 1_fps;
    constexpr units::feet_per_second_squared_t MAX_ACCELERATION = 1_fps_sq;
    constexpr bool INVERT_MOTORS = false;
    constexpr double STALL_LIMIT = 0.9;
    constexpr double STALL_TRIGGER = 0.1;
    constexpr units::unit_t<units::compound_unit<units::inch, units::inverse<units::turn>>> ELEVATOR_RATIO = 1_in/1_tr;
    constexpr units::inch_t POSITION_TOLERANCE = 1_in;
    constexpr units::inch_t HOME_POSITION = 0_in;
    constexpr units::feet_per_second_t HOME_VELOCITY = -0.5_fps;

    constexpr SC::SC_PIDConstants PID_C(0, 0, 0, 0);
    constexpr SC::SC_LinearFeedForward FEED_FORWARD(0_V, 0_V, 0_V / 1_mps, 0_V / 1_mps_sq);
}

namespace IntakeConstants {
        constexpr int MOTOR_CAN_ID = 40;
        constexpr int ALGAE_TOP_SENSOR_DI_CH = 1;
        constexpr int ALGAE_BOTTOM_SENSOR_DI_CH = 2;
        constexpr int CORAL_HIGH_SENSOR_DI_CH = 3;
        constexpr int CORAL_LOW_SENSOR_DI_CH = 5;
}

namespace PivotConstants {
    constexpr int PIVOT_MOTOR_CAN_ID = 41;
    constexpr int PIVOT_HOME_DI_CH = 3;

    constexpr units::radians_per_second_t MAX_VELOCITY = 5_deg_per_s;
    constexpr units::radians_per_second_squared_t MAX_ACCELERATION = 5_deg_per_s_sq;

    constexpr bool INVERT_MOTOR = false;

    constexpr SC::SC_PIDConstants PID_C(0, 0, 0, 0);
    constexpr SC::SC_AngularFeedForward FEED_FORWARD(0_V, 0_V, 0_V / 1_rad_per_s, 0_V / 1_rad_per_s_sq);

    constexpr double STALL_LIMIT = 0.9;
    constexpr double STALL_TRIGGER = 0.1;
    constexpr double GEAR_RATIO = 1/1; // edit later

    constexpr units::degree_t ANGLE_TOLERANCE = 5_deg;
    constexpr units::degree_t HOME_POSITION = 0_deg;
    constexpr units::degree_t TARGET_POSITION = 0_deg;
    constexpr double HOME_POWER = -0.2;
}

#endif
