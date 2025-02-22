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
bool Driver_Interface::GetSetCoastMode() {return _driver_controller.GetRawButtonPressed(TOGGLE_COAST_MODE);}
bool Driver_Interface::LowSpeed() {return _driver_controller.GetRawAxis(LOW_SPEED) > 0.5;}
bool Driver_Interface::GetDynamicPivot() {return _driver_controller.GetRawButton(DYNAMIC_PIVOT);}

bool Driver_Interface::GetCoralPickup() {return _driver_controller.GetRawButton(AUTO_CORAL_PICKUP);}
bool Driver_Interface::GetAlgaePickup() {return _driver_controller.GetRawButton(AUTO_ALGAE_PICKUP);}
bool Driver_Interface::GetScoreReef() {return _driver_controller.GetRawButton(AUTO_SCORE_REEF);}
bool Driver_Interface::GetScoreProcessor() {return _driver_controller.GetRawButton(AUTO_SCORE_PROCESSOR);}

void Driver_Interface::SetRumble(double Rumble) {
    _driver_controller.SetRumble(frc::GenericHID::kBothRumble, Rumble);
}

bool Driver_Interface::DriverOverride(){return _driver_controller.GetPOV(DRIVER_OVERRIDE);}

// ----------
// Operator
// ----------
Operator_Interface::Operator_Interface(){}
void Operator_Interface::SetRumble(double Rumble) {_operator_controller.SetRumble(frc::GenericHID::kBothRumble, Rumble);}
bool Operator_Interface::IgnoreVision(){return false;} /*Make this a button later*/

Alignment Operator_Interface::GetReefAllignmet() {if (CORAL_LEVEL_4_LEFT, CORAL_LEVEL_3_LEFT, CORAL_LEVEL_2_LEFT){return left;} else if (CORAL_LEVEL_4_RIGHT, CORAL_LEVEL_3_RIGHT, CORAL_LEVEL_2_RIGHT){return right;} else {return center;}};
int Operator_Interface::GetReefLevel() {if (CORAL_LEVEL_4_LEFT,CORAL_LEVEL_4_RIGHT){return 4;} else if(CORAL_LEVEL_3_LEFT, CORAL_LEVEL_3_RIGHT, ALGAE_LEVEL_3){return 3;} else if(CORAL_LEVEL_2_LEFT, CORAL_LEVEL_2_RIGHT, ALGAE_LEVEL_2){ return 2;} else if (CORAL_LEVEL_1){return 1;} else {return 0;}}

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