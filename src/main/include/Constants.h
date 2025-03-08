
#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <photon/PhotonPoseEstimator.h>
#include <frc/geometry/Translation2d.h>

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
#include <frc/LEDPattern.h>
#include <vector>

#include "Config.h"

namespace VisionConstants {
    const frc::AprilTagFieldLayout APRIL_TAG_LAYOUT = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2025Reefscape);
    constexpr photon::PoseStrategy POSE_STRATEGY = photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR;

    const std::vector<SC::SC_CameraConfig> CAMERA_CONFIGS = {
        SC::SC_CameraConfig{
            "Camera_1",
            frc::Transform3d{frc::Translation3d{12_in, 0_in, 12_in}, frc::Rotation3d{0_deg, -30_deg, 0_deg}},
        },
        SC::SC_CameraConfig{
            "Camera_2",
            frc::Transform3d{frc::Translation3d{12_in, 0_in, 12_in}, frc::Rotation3d{0_deg, -30_deg, 0_deg}},
        }
    };
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
        static SC::SC_SwerveConfigs SWERVE_FRONT_LEFT{12,13,18, 27.685546875000004_deg - 180_deg}; //-27.685546875000004
        static SC::SC_SwerveConfigs SWERVE_FRONT_RIGHT{10,11,19, -167.16796875_deg}; //167.16796875
        static SC::SC_SwerveConfigs SWERVE_BACK_LEFT{16,17,21, 38.759765625_deg - 180_deg}; //-38.759765625
        static SC::SC_SwerveConfigs SWERVE_BACK_RIGHT{14,15,20, -155.0390625_deg}; //155.0390625

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

        constexpr units::inch_t AT_TARGET_POSITION_THRESHOLD = 6_in;
        constexpr units::inch_t NEAR_TARGET_POSITION_THRESHOLD = 12_in;

        constexpr std::string_view DRIVETRAIN_CANBUS_NAME = "Drivetrain CANivore";
        constexpr int PIGEON_ID = 22;

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

        constexpr int REEF_APRIL_TAGS[] = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
        constexpr frc::Pose2d LEFT_REEF_OFFSET = frc::Pose2d{frc::Translation2d{6_in, 0_in}, frc::Rotation2d{0_deg}};
        constexpr frc::Pose2d CENTER_REEF_OFFSET = frc::Pose2d{frc::Translation2d{0_in, 0_in}, frc::Rotation2d{0_deg}};
        constexpr frc::Pose2d RIGHT_REEF_OFFSET = frc::Pose2d{frc::Translation2d{-6_in, 0_in}, frc::Rotation2d{0_deg}};

        constexpr int FEEDER_STATION_APRIL_TAGS[] = {1, 2, 12, 13};
        // TODO: Fix later when we know where we want to align
        constexpr frc::Pose2d LEFT_FEEDER_STATION_OFFSET = frc::Pose2d{frc::Translation2d{0_in, 0_in}, frc::Rotation2d{180_deg}};
        constexpr frc::Pose2d RIGHT_FEEDER_STATION_OFFSET = frc::Pose2d{frc::Translation2d{-0_in, 0_in}, frc::Rotation2d{180_deg}};

        constexpr int PROCESSOR_APRIL_TAGS[] = {3, 16};
        constexpr frc::Pose2d PROCESSOR_OFFSET = frc::Pose2d{frc::Translation2d{0_in, 0_in}, frc::Rotation2d{0_deg}};
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
        constexpr int BRAKE = XBOX_LB;
        constexpr int TOGGLE_COAST_MODE = XBOX_START;
        constexpr int LOW_SPEED = XBOX_RT;
        constexpr int DYNAMIC_PIVOT = XBOX_RB;

        constexpr int AUTO_CORAL_PICKUP = XBOX_A; //change once dpad exists
        constexpr int AUTO_ALGAE_PICKUP = XBOX_X;
        constexpr int AUTO_SCORE_REEF = XBOX_B;
        constexpr int AUTO_SCORE_PROCESSOR = XBOX_Y;

        // Override
        constexpr int DRIVER_OVERRIDE = XBOX_DPAD_UP;

    }
    namespace Operator {
        #ifdef OPERATOR_BUTTON_BOX
        constexpr int OPERATOR_CONTROLLER_PORT = 1;
        constexpr int CORAL_LEVEL_4_LEFT = 0;
        constexpr int CORAL_LEVEL_4_RIGHT = 0;
        constexpr int CORAL_LEVEL_3_LEFT = 0;
        constexpr int CORAL_LEVEL_3_RIGHT = 0;
        constexpr int CORAL_LEVEL_2_LEFT = 0;
        constexpr int CORAL_LEVEL_2_RIGHT = 0;
        constexpr int CORAL_LEVEL_1 = 0;

        constexpr int ALGAE_LEVEL_3 = 0;
        constexpr int ALGAE_LEVEL_2 = 0;

        constexpr int GROUND = 0;
        constexpr int PROCESSOR = 0;
        constexpr int CLIMB_UP = 0;
        constexpr int CLIMB_DOWN = 0;
        constexpr int NET = 0;
        constexpr int IGNORE_VISION = 0;
        constexpr int LOAD_CORAL = 0;
        
        #else

        constexpr int OPERATOR_CONTROLLER_PORT = 1;
        constexpr double OPERATOR_JOYSTICK_DEADBAND = 0.02;

        constexpr int CORAL_LEVEL_4_LEFT = XBOX_A;
        constexpr int CORAL_LEVEL_4_RIGHT = XBOX_B;
        constexpr int CORAL_LEVEL_3_LEFT = XBOX_X;
        constexpr int CORAL_LEVEL_3_RIGHT = XBOX_Y;
        constexpr int CORAL_LEVEL_2_LEFT = XBOX_LB;
        constexpr int CORAL_LEVEL_2_RIGHT = XBOX_RB;
        constexpr int CORAL_LEVEL_1 = XBOX_DPAD_RIGHT;

        constexpr int ALGAE_LEVEL_3 = XBOX_R3;
        constexpr int ALGAE_LEVEL_2 = XBOX_L3;

        constexpr int GROUND = XBOX_LT;
        constexpr int PROCESSOR = XBOX_RT;
        constexpr int CLIMB_UP = XBOX_DPAD_UP;
        constexpr int CLIMB_DOWN = XBOX_DPAD_DOWN;
        constexpr int NET = XBOX_START;
        constexpr int IGNORE_VISION = XBOX_BACK;
        constexpr int LOAD_CORAL = XBOX_DPAD_LEFT;

        #endif
    }

    namespace Testing {
        constexpr int TESTING_OPEN_LOOP_LEFT = XBOX_LS_Y;
        constexpr int TESTING_OPEN_LOOP_RIGHT = XBOX_RS_Y;
        constexpr int PIVOT_GET_MOTOR = XBOX_RS_Y;
        constexpr int ELEVATOR_GET_MOTOR = XBOX_LS_Y;
        constexpr int INTAKE_GET_FORWARD_MOTOR = XBOX_RT;
        constexpr int INTAKE_GET_BACKWARD_MOTOR = XBOX_LT;
        constexpr double TESTING_DEADBAND = 0.02;
        constexpr int TESTING_CONTROLLER_PORT = 2;
        constexpr double PIVOT_POWER_LIMIT = 0.3;
        constexpr double ELEVATOR_POWER_LIMIT = 0.3;
        constexpr double INTAKE_POWER_LIMIT = 0.6;
    }
}

namespace ElevatorConstants {
    constexpr int PRIMARY_MOTOR_CAN_ID = 30;
    constexpr int SECONDARY_MOTOR_CAN_ID = 31;
    constexpr int HOME_SENSOR_DI_CH = 0;
    constexpr int BRAKE_SERVO = 0;

    constexpr units::feet_per_second_t MAX_VELOCITY = 22.5_in / 1_s;
    constexpr units::feet_per_second_squared_t MAX_ACCELERATION = 33.75_in / 1_s / 1_s;
    constexpr bool INVERT_MOTORS = true;
    constexpr bool MIRROR_MOTORS = true;
    constexpr double STALL_LIMIT = 0.9;
    constexpr double STALL_TRIGGER = 0.1;
    constexpr units::unit_t<units::compound_unit<units::inch, units::inverse<units::turn>>> ELEVATOR_RATIO = 0.505_in/1_tr;
    constexpr units::inch_t POSITION_TOLERANCE = 1_in;

    constexpr units::feet_per_second_t HOME_VELOCITY = -0.5_fps;

    constexpr double RATCHET_ENGAGED = 1.0;
    constexpr double RATCHET_DISENGAGED = 0.0; 

    // P: 61.605, I: 0, D: 16.759
    constexpr SC::SC_PIDConstants PID_C(0.125, 0, 16.759, 0);
    constexpr SC::SC_LinearFeedForward FEED_FORWARD(0.43085_V, 010705_V, 9.204_V / 1_mps, 4.3885_V / 1_mps_sq);

    // Elevator positions
    constexpr units::inch_t HOME_POSITION = 0_in;
    constexpr units::inch_t PROCESSOR_POSITION = 0_in;
    constexpr units::inch_t CLIMB_HEIGHT = 0_in;
    
    constexpr units::inch_t CORAL_LEVEL_1 = 0_in;
    constexpr units::inch_t CORAL_LEVEL_2 = 0_in;
    constexpr units::inch_t CORAL_LEVEL_3 = 0_in;
    constexpr units::inch_t CORAL_LEVEL_4 = 0_in;

    constexpr units::inch_t ALGAE_LEVEL_2 = 0_in;
    constexpr units::inch_t ALGAE_LEVEL_3 = 0_in;
}

namespace IntakeConstants {
        constexpr int MOTOR_CAN_ID = 40;
        constexpr int ALGAE_TOP_SENSOR_DI_CH = 3;
        constexpr int ALGAE_BOTTOM_SENSOR_DI_CH = 5;
        constexpr int CORAL_HIGH_SENSOR_DI_CH = 1;
        constexpr int CORAL_LOW_SENSOR_DI_CH = 2;

        constexpr double EJECT_POWER = 1.0;
        constexpr double STOP_POWER = 0.0;
        constexpr double INTAKE_POWER = 2.0;

        constexpr bool INVERT_MOTOR = false;
        
        // constexpr int ROLLER_STOP = 0;
        // constexpr double ROLLER_EJECT = -1.0;
        // constexpr double ROLLER_INTAKE = 0.4;
}

namespace PivotConstants {
    constexpr int PIVOT_MOTOR_CAN_ID = 41;
    constexpr int PIVOT_HOME_DI_CH = 4;

    constexpr units::radians_per_second_t MAX_VELOCITY = 67.5_deg_per_s;
    constexpr units::radians_per_second_squared_t MAX_ACCELERATION = 101.25_deg_per_s_sq;

    constexpr bool INVERT_MOTOR = false;
    
    // P: 24.275, I: 0, D: 5.9465
    constexpr SC::SC_PIDConstants PID_C(0.05, 0, 0, 0);
    constexpr SC::SC_AngularFeedForward FEED_FORWARD(0.75016_V, 0.38962_V, 3.6757_V / 1_rad_per_s, 1.2884_V / 1_rad_per_s_sq);

    constexpr double STALL_LIMIT = 0.9;
    constexpr double STALL_TRIGGER = 0.1;
    constexpr double GEAR_RATIO = 45/1; // edit later

    constexpr units::degree_t ANGLE_TOLERANCE = 5_deg;
    constexpr units::degree_t HOME_POSITION = 102.5_deg;
    constexpr units::degree_t PROCESSOR_POSITION = 0_deg;
    constexpr units::degree_t TRAVEL_POSITION = 0_deg;
    constexpr units::degree_t INTAKE_POSITION = 0_deg;
    constexpr units::degree_t TARGET_CORAL_ANGLE = 0_deg;
    constexpr units::degree_t TARGET_CORAL_4_ANGLE = 0_deg;
    constexpr units::degree_t TARGET_ALGAE_ANGLE = 0_deg;
    constexpr double HOME_POWER = -0.2;
}

namespace FunnelConstants {
    constexpr int MOTOR_CAN_ID = 50;
    constexpr int CORAL_SENSOR_DI_CH = 6;

    constexpr double STOP_POWER = 0.0;
    constexpr double INTAKE_POWER = 0.5;
}

namespace LEDConstants {
    constexpr int LED_PWM_PORT = 9;
    constexpr int LED_STRIP_LENGTH = 260;

    constexpr units::meter_t LED_SPACING = 1_m / 60.0;
    constexpr units::meter_t WAVELENGTH = 0.25_m;
    constexpr units::meters_per_second_t SCROLLING_SPEED = 0.25_mps;
    constexpr frc::Color ALGAE_GREEN {"#10F01A"};
    constexpr frc::Color TEAM_BLUE {"#009BB4"};
    constexpr frc::Color DRIVE_ORANGE {"#FF8200"};
    constexpr frc::Color CORAL_PINK {"#FF0091"};
    const std::vector<frc::Color> COLORS = {TEAM_BLUE, DRIVE_ORANGE, CORAL_PINK};
    constexpr double GAMMA = 2.2;
    constexpr units::second_t PIVOT_ANIMATION_TIME = 0.8_s;
    constexpr int BAR_SIZE = 1;
    constexpr units::meters_per_second_t VELOCITY = 0.5_mps;
    constexpr units::meters_per_second_squared_t EXIT_ACCELERATION = 0.5_mps_sq;
    constexpr size_t FILL_SIZE = 5;
    constexpr size_t EMPTY_SIZE = 1;
    constexpr int FIRE_HEIGHT = 1;
    constexpr int SPARKS = 5;
    constexpr int DELAY = 1;
}

#endif
