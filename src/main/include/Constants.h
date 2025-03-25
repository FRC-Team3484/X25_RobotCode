
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
#include <units/constants.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <frc/LEDPattern.h>
#include <vector>

#include "Config.h"

namespace VisionConstants {
    const frc::AprilTagFieldLayout APRIL_TAG_LAYOUT = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2025ReefscapeWelded);
    constexpr photon::PoseStrategy POSE_STRATEGY = photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR;

    const Eigen::Matrix<double, 3, 1> SINGLE_TAG_STDDEV{4, 4, 8};
    const Eigen::Matrix<double, 3, 1> MULTI_TAG_STDDEV{0.5, 0.5, 1};

    const std::vector<SC::SC_CameraConfig> CAMERA_CONFIGS = {
        SC::SC_CameraConfig{
            "Camera_1",
            frc::Transform3d{frc::Translation3d{10.3_in, 11.62_in, 4.75_in}, frc::Rotation3d{0_deg, -45_deg, -49.63_deg}},
            true
        },
        SC::SC_CameraConfig{
            "Camera_2",
            frc::Transform3d{frc::Translation3d{-9.82_in, -11.68_in, 4.8_in}, frc::Rotation3d{0_deg, -65_deg, 180_deg - 37.17_deg}},
            false
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
        static SC::SC_SwerveConfigs SWERVE_FRONT_LEFT{12,13,18, 27.685546875000004_deg}; //-27.685546875000004
        static SC::SC_SwerveConfigs SWERVE_FRONT_RIGHT{10,11,19, -167.16796875_deg + 180_deg}; //167.16796875
        static SC::SC_SwerveConfigs SWERVE_BACK_LEFT{16,17,21, 38.759765625_deg}; //-38.759765625
        static SC::SC_SwerveConfigs SWERVE_BACK_RIGHT{14,15,20, -155.0390625_deg + 180_deg}; //155.0390625

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
            constexpr double JOG_SCALE = 0.25;
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
        constexpr frc::Pose2d LEFT_REEF_OFFSET = frc::Pose2d{frc::Translation2d{22_in, -7_in}, frc::Rotation2d{0_deg}};
        constexpr frc::Pose2d CENTER_REEF_OFFSET = frc::Pose2d{frc::Translation2d{22_in, 0_in}, frc::Rotation2d{0_deg}};
        constexpr frc::Pose2d RIGHT_REEF_OFFSET = frc::Pose2d{frc::Translation2d{22_in, 7_in}, frc::Rotation2d{0_deg}};

        constexpr int FEEDER_STATION_APRIL_TAGS[] = {1, 2, 12, 13};
        // TODO: Fix later when we know where we want to align
        constexpr frc::Pose2d LEFT_FEEDER_STATION_OFFSET = frc::Pose2d{frc::Translation2d{22_in, -24_in}, frc::Rotation2d{180_deg}};
        constexpr frc::Pose2d RIGHT_FEEDER_STATION_OFFSET = frc::Pose2d{frc::Translation2d{22_in, 24_in}, frc::Rotation2d{180_deg}};

        constexpr int PROCESSOR_APRIL_TAGS[] = {3, 16};
        constexpr frc::Pose2d PROCESSOR_OFFSET = frc::Pose2d{frc::Translation2d{22_in, 0_in}, frc::Rotation2d{0_deg}};

        constexpr frc::Pose2d STARTING_POSITION_A = frc::Pose2d{frc::Translation2d{48_in, 24_in}, frc::Rotation2d{0_deg}};
        constexpr frc::Pose2d STARTING_POSITION_B = frc::Pose2d{frc::Translation2d{0_in, 0_in}, frc::Rotation2d{0_deg}};
        constexpr frc::Pose2d STARTING_POSITION_C = frc::Pose2d{frc::Translation2d{0_in, 0_in}, frc::Rotation2d{0_deg}};
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

    }
    namespace Operator {
        #ifdef OPERATOR_BUTTON_BOX
        constexpr int OPERATOR_CONTROLLER_PORT = 1;
        constexpr int CORAL_LEVEL_4 = 17;
        //constexpr int CORAL_LEVEL_4_RIGHT = 8;
        constexpr int CORAL_LEVEL_3 = 19;
        //constexpr int CORAL_LEVEL_3_RIGHT = 7;
        constexpr int CORAL_LEVEL_2 = 20;
        //constexpr int CORAL_LEVEL_2_RIGHT = 21; //6
        constexpr int CORAL_LEVEL_1 = 11;

        constexpr int ALGAE_LEVEL_3 = 16;
        constexpr int ALGAE_LEVEL_2 = 9;

        //constexpr int CONFIRM_MANUAL_SCORE_TWO = 5;
        constexpr int CONFIRM_MANUAL_SCORE = 6; //21
        //constexpr int CLIMB_UP = 14;
        //constexpr int CLIMB_DOWN = 18;
        constexpr int IGNORE_VISION = 2;
        constexpr int LOAD_CORAL = 4;
        constexpr int RESET = 13;
        
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
        constexpr int IGNORE_VISION = XBOX_START;
        constexpr int LOAD_CORAL = XBOX_DPAD_LEFT;
        constexpr int CONFIRM_MANUAL_SCORE = XBOX_BACK;

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
    //constexpr int BRAKE_SERVO = 0;

    constexpr bool INVERT_MOTORS = true;
    constexpr bool MIRROR_MOTORS = true;
    constexpr double STALL_LIMIT = 0.9;
    constexpr double STALL_TRIGGER = 0.1;
    constexpr units::unit_t<units::compound_unit<units::inch, units::inverse<units::turn>>> ELEVATOR_RATIO = units::inch_t{units::constants::pi.value()/5.0}/1_tr; // 0.505_in
    constexpr units::inch_t POSITION_TOLERANCE = 0.2_in;

    constexpr units::feet_per_second_t HOME_VELOCITY = -0.2_fps;

    constexpr double RATCHET_ENGAGED = 1.0;
    constexpr double RATCHET_DISENGAGED = 0.0; 

    constexpr units::feet_per_second_t MAX_VELOCITY = 20_in / 1_s;
    constexpr units::feet_per_second_squared_t MAX_ACCELERATION = 720_in / 1_s / 1_s;
    // P: 61.605, I: 0, D: 16.759
    constexpr SC::SC_PIDConstants PID_C(0.8, 0.1, 0, 0);
    constexpr SC::SC_LinearFeedForward FEED_FORWARD(0.18_V, 0.42_V, 0.0_V / 1_mps, 0.0_V / 1_mps_sq);

    // Elevator positions
    constexpr units::inch_t HOME_POSITION = 0_in;
    constexpr units::inch_t PROCESSOR_POSITION = 6_in;
    constexpr units::inch_t CLIMB_HEIGHT = 18_in;
    constexpr units::inch_t INTAKE_HEIGHT = 1.25_in;
    
    constexpr units::inch_t CORAL_LEVEL_1 = 15_in;
    constexpr units::inch_t CORAL_LEVEL_2 = 23.5_in;
    constexpr units::inch_t CORAL_LEVEL_3 = 39_in;
    constexpr units::inch_t CORAL_LEVEL_4 = 58.5_in;

    constexpr units::inch_t ALGAE_LEVEL_2 = 20_in;
    constexpr units::inch_t ALGAE_LEVEL_3 = 30_in;

    constexpr units::inch_t SAFE_STOW_POSITION = 5_in;
    constexpr units::inch_t EXTENDED_POSITION = 30_in;
}

namespace IntakeConstants {
        constexpr int MOTOR_CAN_ID = 40;
        constexpr int ALGAE_TOP_SENSOR_DI_CH = 3;
        constexpr int ALGAE_BOTTOM_SENSOR_DI_CH = 5;
        constexpr int CORAL_HIGH_SENSOR_DI_CH = 1;
        constexpr int CORAL_LOW_SENSOR_DI_CH = 2;

        constexpr double CORAL_EJECT_POWER = -1.0;
        constexpr double ALGAE_EJECT_POWER = 1.0;
        constexpr double STOP_POWER = 0.0;
        constexpr double INTAKE_POWER = -1.0;

        constexpr bool INVERT_MOTOR = false;
        
        // constexpr int ROLLER_STOP = 0;
        // constexpr double ROLLER_EJECT = -1.0;
        // constexpr double ROLLER_INTAKE = 0.4;
}

namespace PivotConstants {
    constexpr int PIVOT_MOTOR_CAN_ID = 41;
    constexpr int PIVOT_HOME_DI_CH = 4;

    constexpr units::radians_per_second_t MAX_VELOCITY = 80_deg_per_s; //67.5_deg_per_s; 
    constexpr units::radians_per_second_squared_t MAX_ACCELERATION = 480_deg_per_s_sq; //101.25_deg_per_s_sq;

    constexpr bool INVERT_MOTOR = false;
    
    // P: 24.275, I: 0, D: 5.9465
    constexpr SC::SC_PIDConstants PID_C(0.200, 0.1, 0, 0);
    constexpr SC::SC_AngularFeedForward FEED_FORWARD(0.6_V, 0.36_V, 0_V / 1_rad_per_s, 0_V / 1_rad_per_s_sq); // 0.28_V, 0.52_V, 3.6757_V / 1_rad_per_s, 1.2884_V / 1_rad_per_s_sq);

    constexpr double STALL_LIMIT = 0.9;
    constexpr double STALL_TRIGGER = 0.1;
    constexpr double GEAR_RATIO = 45/1;

    // All functional angles are 90 degrees more, straight up (aligned with elevator sides) is 90 degrees
    constexpr units::degree_t ANGLE_TOLERANCE = 7.5_deg;
    constexpr units::degree_t HOME_POSITION = 102.5_deg;
    constexpr units::degree_t PROCESSOR_POSITION = 140_deg;
    constexpr units::degree_t TRAVEL_POSITION = 140_deg;
    constexpr units::degree_t INTAKE_POSITION = 122_deg;
    constexpr units::degree_t TARGET_CORAL_ANGLE = 140_deg;
    constexpr units::degree_t TARGET_CORAL_4_ANGLE = 150_deg;
    constexpr units::degree_t TARGET_ALGAE_ANGLE = 150_deg;
    constexpr double HOME_POWER = -0.1;
}

namespace FunnelConstants {
    constexpr int MOTOR_CAN_ID = 50;
    constexpr int CORAL_SENSOR_DI_CH = 6;

    constexpr double STOP_POWER = 0.0;
    constexpr double INTAKE_POWER = 0.5;
}

namespace LEDConstants {
    constexpr int LED_PWM_PORT = 1;
    constexpr int LED_STRIP_LENGTH = 72;

    constexpr units::meter_t LED_SPACING = 1_m / 60.0;
    constexpr units::meter_t WAVELENGTH = 0.25_m;
    constexpr units::meters_per_second_t SCROLLING_SPEED = 0.25_mps;
    constexpr frc::Color ALGAE_GREEN {"#10F01A"};
    constexpr frc::Color TEAM_BLUE {"#009BB4"};
    constexpr frc::Color DRIVE_ORANGE {"#FF8200"};
    constexpr frc::Color CORAL_PINK {"#FF0091"};
    constexpr frc::Color FIRE_RED {frc::Color::kRed};
    const std::vector<frc::Color> COLORS = {DRIVE_ORANGE, TEAM_BLUE, CORAL_PINK};
    constexpr double GAMMA = 2.2;
    constexpr units::second_t PIVOT_ANIMATION_TIME = 0.8_s;
    constexpr int BAR_SIZE = 12;
    constexpr units::meters_per_second_t VELOCITY = 0.5_mps;
    constexpr units::meters_per_second_squared_t EXIT_ACCELERATION = 0.5_mps_sq;
    constexpr size_t FILL_SIZE = 2;
    constexpr size_t EMPTY_SIZE = 2;
    constexpr int FIRE_HEIGHT = 1;
    constexpr int SPARKS = 2;
    constexpr int DELAY = 1;
    constexpr units::second_t SCORING_BLUE_ON_TIME = 0.6_s;
    constexpr units::second_t SCORING_BLUE_OFF_TIME = 0.2_s;
    constexpr units::second_t LOW_BATTERY_CYCLE_TIME = 2_s;
    constexpr units::second_t ELEVATOR_HOME_CYCLE_TIME = 1.5_s;
}

#endif
