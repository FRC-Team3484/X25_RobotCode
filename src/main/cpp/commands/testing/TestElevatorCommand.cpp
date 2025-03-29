#include "commands/testing/TestElevatorCommand.h"
#include <frc/smartdashboard/SmartDashboard.h>

#include <units/math.h>

TestElevatorCommand::TestElevatorCommand(ElevatorSubsystem* elevator_subsystem, Testing_Interface* testing_interface)
: _elevator_subsystem(elevator_subsystem), _testing_interface(testing_interface) {
    AddRequirements({_elevator_subsystem});
}

void TestElevatorCommand::Initialize() {
    _elevator_subsystem->SetTestMode(true);
    frc::SmartDashboard::PutBoolean("Test Elevator", false);
}

void TestElevatorCommand::Execute() {
    _elevator_subsystem->PrintTestInfo();

    if (frc::SmartDashboard::GetBoolean("Test Elevator", false)) {
        if (_testing_interface->GetA()){
            _elevator_subsystem->SetHeight(ElevatorConstants::CORAL_LEVEL_4);
        } else  if (std::abs(_testing_interface->GetRawElevator()) > 0.0) {
            _elevator_subsystem->SetPower(_testing_interface->GetRawElevator());
        }
        else {
            _elevator_subsystem->SetHeight(ElevatorConstants::HOME_POSITION);
        }
    }
}

void TestElevatorCommand::End(bool interrupted) {
    _elevator_subsystem->SetTestMode(false);
}

bool TestElevatorCommand::IsFinished() {
    return false;
}
