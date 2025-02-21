#ifndef OI_H
#define OI_H

#include "Constants.h"
#include <frc/XboxController.h>

/**
 * Describes the actions used on the driver's controller
 */
class Driver_Interface {
    public:
        Driver_Interface();
        //  Swerve Controllers
        double GetThrottle();
        double GetStrafe();
        double GetRotation();

        bool GetResetHeading();
        bool GetBrake();
        bool GetBrakePressed();

        bool GetSetCoastMode();
        
        bool GetDynamicPivot();

        bool LowSpeed();
        void SetRumble(double Rumble);

        bool DriverOverride();

        bool GetCoralPickup();
        bool GetAlgaePickup();
        bool GetScoreReef();
        bool GetScoreProcessor();


    private:
        frc::XboxController _driver_controller{UserInterface::Driver::DRIVER_CONTROLLER_PORT};
};

/**
 * Describes the actions used on the operator's controller or button box
 */
class Operator_Interface {
    public:
        Operator_Interface();

        void SetRumble(double Rumble);

        bool GetAlgaeLevel4Left();
        bool GetAlgaeLevel4Right();
        bool GetAlgaeLevel3Left();
        bool GetAlgaeLevel3Right();
        bool GetAlgaeLevel2Left();
        bool GetAlgaeLevel2Right();
        bool GetAlgaeLevel1();
        bool GetCoralLevel3();
        bool GetCoralLevel2();
        bool GetGround();
        bool GetProcessor();
        bool GetClimbUp();
        bool GetClimbDown();
        bool GetNet();
        bool GetIgnoreVision();

        int RawPOV();

        bool IgnoreVision();

    private:
        frc::GenericHID _operator_controller{UserInterface::Operator::OPERATOR_CONTROLLER_PORT};
};

/**
 * Describes the actions used when the robot is in test mode
 */
class Testing_Interface {
    public:
        Testing_Interface();

        double GetMotor1();

    private:
        frc::XboxController _testing_controller{UserInterface::Testing::TESTING_CONTROLLER_PORT};
};

#endif