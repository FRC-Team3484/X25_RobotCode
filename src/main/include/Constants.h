#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <FRC3484_Lib/utils/SC_ControllerMaps.h>
#include <FRC3484_Lib/utils/SC_Datatypes.h>

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
        constexpr double TESTING_DEADBAND = 0.02;
        constexpr int TESTING_CONTROLLER_PORT = 2;
    }
}

// We changed PivotConstants to Pivot due to this is how elevator(constants) was named
namespace Pivot {
    constexpr int PIVOT_MOTOR_CAN_ID = 16;
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
