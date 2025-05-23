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
bool Driver_Interface::LowSpeed() {return _driver_controller.GetRawAxis(LOW_SPEED) > 0.5;}
bool Driver_Interface::GetDynamicPivot() {return _driver_controller.GetRawButton(DYNAMIC_PIVOT);}

bool Driver_Interface::GetCoralPickup() {return _driver_controller.GetRawButton(AUTO_CORAL_PICKUP);}
bool Driver_Interface::GetAlgaePickup() {return _driver_controller.GetRawButton(AUTO_ALGAE_PICKUP);}
bool Driver_Interface::GetScoreReef() {return _driver_controller.GetRawButton(AUTO_SCORE_REEF);}
bool Driver_Interface::GetScoreProcessor() {return _driver_controller.GetRawButton(AUTO_SCORE_PROCESSOR);}
// bool Driver_Interface::GetCoralPickup() {return false;}
// bool Driver_Interface::GetAlgaePickup() {return false;}
// bool Driver_Interface::GetScoreReef() {return false;}
// bool Driver_Interface::GetScoreProcessor() {return false;}

int Driver_Interface::RawPOV() {return _driver_controller.GetPOV();}

void Driver_Interface::SetRumble(double Rumble) {
    _driver_controller.SetRumble(frc::GenericHID::kBothRumble, Rumble);
}


// ----------
// Operator
// ----------
Operator_Interface::Operator_Interface(){}
#ifdef OPERATOR_BUTTON_BOX 

void Operator_Interface::SetRumble(double Rumble) {_operator_controller.SetRumble(frc::GenericHID::kBothRumble, Rumble);}
bool Operator_Interface::IgnoreVision(){return _operator_controller.GetRawButton(IGNORE_VISION);} /*Make this a button later*/

int Operator_Interface::GetReefLevel() {if (_operator_controller.GetRawButton(CORAL_LEVEL_4)) {
        return 4;
    } else if(_operator_controller.GetRawButton(CORAL_LEVEL_3)  || _operator_controller.GetRawButton(ALGAE_LEVEL_3)) {
        return 3;
    } else if(_operator_controller.GetRawButton(CORAL_LEVEL_2) || _operator_controller.GetRawButton(ALGAE_LEVEL_2)) {
        return 2;
    } else if (_operator_controller.GetRawButton(CORAL_LEVEL_1)) {
        return 1;
    } else {
        return 0;
    }
}

bool Operator_Interface::GetAlgaeOrCoral() {
    if (_operator_controller.GetRawButton(ALGAE_LEVEL_3) || _operator_controller.GetRawButton(ALGAE_LEVEL_2)) {
        return false;
    } else {
        return true;
    }
}

bool Operator_Interface::GetConfirmManualScore() {return _operator_controller.GetRawButton(CONFIRM_MANUAL_SCORE) || _operator_controller.GetRawButton(CONFIRM_MANUAL_SCORE_TWO);}
bool Operator_Interface::GetIgnoreVision() {return _operator_controller.GetRawButton(IGNORE_VISION);}
bool Operator_Interface::GetLoadCoral() {return _operator_controller.GetRawButton(LOAD_CORAL) || _operator_controller.GetRawButton(LOAD_CORAL_TWO);}
bool Operator_Interface::GetReset() {return _operator_controller.GetRawButton(RESET);}

int Operator_Interface::RawPOV() {return _operator_controller.GetPOV();}

#else

void Operator_Interface::SetRumble(double Rumble) {_operator_controller.SetRumble(frc::GenericHID::kBothRumble, Rumble);}
#ifdef VISION_ENABLED
bool Operator_Interface::IgnoreVision(){return _operator_controller.GetRawButton(IGNORE_VISION);} /* Make this a button laser*/
#else
bool Operator_Interface::IgnoreVision(){return true}
#endif

ReefAlignment Operator_Interface::GetReefAlignment() {
    if (_operator_controller.GetRawButton(CORAL_LEVEL_4_LEFT) || _operator_controller.GetRawButton(CORAL_LEVEL_3_LEFT) || _operator_controller.GetRawButton(CORAL_LEVEL_2_LEFT)) {
        return ReefAlignemnt::left;
    } else if (_operator_controller.GetRawButton(CORAL_LEVEL_4_RIGHT) || _operator_controller.GetRawButton(CORAL_LEVEL_3_RIGHT) || _operator_controller.GetRawButton(CORAL_LEVEL_2_RIGHT)) {
        return ReefAlignemnt::right;
    } else {
        return ReefAlignment::center;
    }
}
int Operator_Interface::GetReefLevel() {
    if (_operator_controller.GetRawButton(CORAL_LEVEL_4_LEFT) || _operator_controller.GetRawButton(CORAL_LEVEL_4_RIGHT)) {
        return 4;
    } else if(_operator_controller.GetRawButton(CORAL_LEVEL_3_LEFT) || _operator_controller.GetRawButton(CORAL_LEVEL_3_RIGHT)){
        return 3;
    } else if(_operator_controller.GetRawButton(CORAL_LEVEL_2_LEFT) || _operator_controller.GetRawButton(CORAL_LEVEL_2_RIGHT) || _operator_controller.GetRawButton(ALGAE_LEVEL_2)){ 
        return 2;
    } else if (_operator_controller.GetPOV() == CORAL_LEVEL_1) {
        return 1;
    } else {
        return 0;
    }
}
bool Operator_Interface::GetClimbUp() {return _operator_controller.GetPOV() == CLIMB_UP;}
bool Operator_Interface::GetClimbDown() {return _operator_controller.GetPOV() == CLIMB_DOWN;}
bool Operator_Interface::GetConfirmManualScore() {return _operator_controller.GetRawButton(CONFIRM_MANUAL_SCORE) || _operator_controller.GetRawButton(CONFIRM_MANUAL_SCORE_TWO);}
bool Operator_Interface::GetIgnoreVision() {return _operator_controller.GetRawButton(IGNORE_VISION);}
bool Operator_Interface::GetLoadCoral() {return _operator_controller.GetPOV() == LOAD_CORAL;}
bool Operator_Interface::GetReset() {return _operator_controller.GetRawButton(RESET);}

int Operator_Interface::RawPOV() {return _operator_controller.GetPOV();}

#endif


// ----------
// Testing
// ----------
Testing_Interface::Testing_Interface() {}

double Testing_Interface::GetRawPivot() {return PIVOT_POWER_LIMIT*frc::ApplyDeadband(_testing_controller.GetRawAxis(PIVOT_GET_MOTOR), TESTING_DEADBAND);}
double Testing_Interface::GetRawElevator() {return ELEVATOR_POWER_LIMIT*frc::ApplyDeadband(_testing_controller.GetRawAxis(ELEVATOR_GET_MOTOR), TESTING_DEADBAND);}
double Testing_Interface::GetRawIntake() {return INTAKE_POWER_LIMIT*(_testing_controller.GetRawAxis(INTAKE_GET_FORWARD_MOTOR) - _testing_controller.GetRawAxis(INTAKE_GET_BACKWARD_MOTOR));}


bool Testing_Interface::GetA() {return _testing_controller.GetRawButton(XBOX_A);}
bool Testing_Interface::GetX() {return _testing_controller.GetRawButton(XBOX_X);}
bool Testing_Interface::GetB() {return _testing_controller.GetRawButton(XBOX_B);}
bool Testing_Interface::GetY() {return _testing_controller.GetRawButton(XBOX_Y);}