#ifndef SC_BUTTONBOX_SS_H
#define SC_BUTTONBOX_SS_H

/*
	Operator Interface Subsystem Class
*/

#include "frc2/command/SubsystemBase.h"
#include "frc2/command/Command.h"
#include "frc2/command/CommandHelper.h"

#include "frc2/command/button/Trigger.h"
#include "frc2/command/button/JoystickButton.h"

#include "frc/GenericHID.h"

namespace SC
{
	class SC_OperatorInput : public frc2::SubsystemBase
	{
	public:
		SC_OperatorInput(int Usb_Port);
		~SC_OperatorInput();

		frc2::JoystickButton GetRawButton(int buttonIdx);
		bool IsButtonPressed(int buttonIdx);

		double GetAxis(int axisIdx);

	private:
		frc::GenericHID gHID;

	};
}

#endif