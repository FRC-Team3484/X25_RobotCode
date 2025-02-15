#include "OI.h"
#include <frc/MathUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace UserInterface::Driver;
using namespace UserInterface::Operator;
using namespace UserInterface::Testing;

// ----------
// Driver
// ----------

//     Motion Drive
Driver_Interface::Driver_Interface(){};
double Driver_Interface::GetThrottle() {return frc::ApplyDeadband(_driver_controller.GetRawAxis(THROTTLE), DRIVER_JOYSTICK_DEADBAND);}
double Driver_Interface::GetStrafe() {return frc::ApplyDeadband(_driver_controller.GetRawAxis(STRAFE), DRIVER_JOYSTICK_DEADBAND);}
double Driver_Interface::GetRotation() {return frc::ApplyDeadband(_driver_controller.GetRawAxis(ROTATION), DRIVER_JOYSTICK_DEADBAND);}
//     Settings Drive
bool Driver_Interface::GetResetHeading() {return _driver_controller.GetRawButton(RESET_HEADING);}
bool Driver_Interface::GetBrake() {return _driver_controller.GetRawButton(BRAKE);}
bool Driver_Interface::GetSetBrakeMode() {return _driver_controller.GetRawButtonPressed(BRAKE_MODE);}
bool Driver_Interface::GetDisableBrakeMode() {return _driver_controller.GetRawButtonPressed(DISABLE_BRAKE_MODE);}
bool Driver_Interface::LowSpeed() {return _driver_controller.GetRawAxis(LOW_SPEED) > 0.5;}
bool Driver_Interface::GetDynamicPivot() {return _driver_controller.GetRawButton(DYNAMIC_PIVOT);}

void Driver_Interface::SetRumble(double Rumble) {
    _driver_controller.SetRumble(frc::GenericHID::kBothRumble, Rumble);
}

// ----------
// Operator
// ----------
Operator_Interface::Operator_Interface(){}
void Operator_Interface::SetRumble(double Rumble) {
    _operator_controller.SetRumble(frc::GenericHID::kBothRumble, Rumble);
}

bool Operator_Interface::GetAlgaeLevel4Left() {return _operator_controller.GetRawButton(ALGAE_LEVEL_4_LEFT);}
bool Operator_Interface::GetAlgaeLevel4Right() {return _operator_controller.GetRawButton(ALGAE_LEVEL_4_RIGHT);}
bool Operator_Interface::GetAlgaeLevel3Left() {return _operator_controller.GetRawButton(ALGAE_LEVEL_3_LEFT);}
bool Operator_Interface::GetAlgaeLevel3Right() {return _operator_controller.GetRawButton(ALGAE_LEVEL_3_RIGHT);}
bool Operator_Interface::GetAlgaeLevel2Left() {return _operator_controller.GetRawButton(ALGAE_LEVEL_2_LEFT);}
bool Operator_Interface::GetAlgaeLevel2Right() {return _operator_controller.GetRawButton(ALGAE_LEVEL_2_RIGHT);}
bool Operator_Interface::GetAlgaeLevel1() {return _operator_controller.GetRawButton(ALGAE_LEVEL_1);}
bool Operator_Interface::GetCoralLevel3() {return _operator_controller.GetRawButton(CORAL_LEVEL_3);}
bool Operator_Interface::GetCoralLevel2() {return _operator_controller.GetRawButton(CORAL_LEVEL_2);}
bool Operator_Interface::GetGround() {return _operator_controller.GetRawButton(GROUND);}
bool Operator_Interface::GetProcessor() {return _operator_controller.GetRawButton(PROCESSOR);}
bool Operator_Interface::GetClimbUp() {return _operator_controller.GetRawButton(CLIMB_UP);}
bool Operator_Interface::GetClimbDown() {return _operator_controller.GetRawButton(CLIMB_DOWN);}
bool Operator_Interface::GetNet() {return _operator_controller.GetRawButton(NET);}
bool Operator_Interface::GetIgnoreVision() {return _operator_controller.GetRawButton(IGNORE_VISION);}

int Operator_Interface::RawPOV() {

    return _operator_controller.GetPOV();
}

// ----------
// Testing
// ----------
Testing_Interface::Testing_Interface() {}

double Testing_Interface::GetMotor1() {return frc::ApplyDeadband(_testing_controller.GetRawAxis(TESTING_GET_MOTOR), TESTING_DEADBAND);}