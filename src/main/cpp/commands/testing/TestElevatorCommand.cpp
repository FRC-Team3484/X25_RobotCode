#include "commands/testing/TestElevatorCommand.h"
#include <frc/smartdashboard/SmartDashboard.h>

TestElevatorCommand::TestElevatorCommand(ElevatorSubsystem* elevator_subsystem, Testing_Interface* testing_interface)
: _elevator_subsystem(elevator_subsystem), _testing_interface(testing_interface) {
    AddRequirements({_elevator_subsystem});
}

void TestElevatorCommand::Initialize() {
    _elevator_subsystem->SetTestMode(true);
    frc::SmartDashboard::PutBoolean("Test Elevator", false);
}

void TestElevatorCommand::Execute() {
    if (frc::SmartDashboard::GetBoolean("Test Elevator", false)) {
        _elevator_subsystem->SetPower(_testing_interface->GetRawElevator());
    }
}

void TestElevatorCommand::End(bool interrupted) {
    _elevator_subsystem->SetTestMode(false);
}

bool TestElevatorCommand::IsFinished() {
    return false;
}
