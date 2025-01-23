#ifndef CONSTANTS_H
#define CONSTANTS_H

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
        constexpr int TESTING_GET_MOTOR_ONE = XBOX_LS_Y;
        constexpr int TESTING_GET_MOTOR_TWO = XBOX_RS_Y;
        constexpr double TESTING_DEADBAND = 0.02;
        constexpr int TESTING_CONTROLLER_PORT = 2;
    }
}

namespace ElevatorConstants {
    constexpr int PRIMARY_MOTOR_CAN_ID = 17;
    constexpr int SECONDARY_MOTOR_CAN_ID = 18;
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
        constexpr int MOTOR_ONE_CAN_ID = 14;
        constexpr int MOTOR_TWO_CAN_ID = 15;
        constexpr int ALGAE_SENSOR_DI_CH = 1;
        constexpr int CORAL_SENSOR_DI_CH = 2;
}

#endif
