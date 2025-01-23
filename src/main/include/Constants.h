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
        constexpr int TESTING_GET_MOTOR_1 = XBOX_LS_Y;
        constexpr int TESTING_GET_MOTOR_2 = XBOX_RS_Y;
        constexpr double TESTING_DEADBAND = 0.02;
        constexpr int TESTING_CONTROLLER_PORT = 2;
    }
}

namespace IntakeConstants {
        constexpr int MOTOR_ONE_CAN_ID = 14;
        constexpr int MOTOR_TWO_CAN_ID = 15;
        constexpr int ALGAE_SENSOR_DI_CH = 1;
        constexpr int CORAL_SENSOR_DI_CH = 2;
}

#endif
