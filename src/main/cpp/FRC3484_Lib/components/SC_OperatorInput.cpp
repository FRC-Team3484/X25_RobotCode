#include "FRC3484_Lib/components/SC_OperatorInput.h"

using namespace SC;
using namespace frc;

SC_OperatorInput::SC_OperatorInput(int Usb_Port) : gHID(Usb_Port) {}

SC_OperatorInput::~SC_OperatorInput() {}

frc2::JoystickButton SC_OperatorInput::GetRawButton(int buttonIdx)
{
	return frc2::JoystickButton(&this->gHID, buttonIdx);
}

bool SC_OperatorInput::IsButtonPressed(int buttonIdx)
{
	return this->gHID.GetRawButton(buttonIdx);
}

double SC_OperatorInput::GetAxis(int axisIdx)
{
	return this->gHID.GetRawAxis(axisIdx);
}