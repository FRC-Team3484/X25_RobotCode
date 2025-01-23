#include "commands/testing/TestElevatorCommand.h"
#include <frc/smartdashboard/SmartDashboard.h>

TestElevatorCommand::TestElevatorCommand(ElevatorSubsystem* elevator_subsystem, Testing_Interface* testing_interface)
: _elevator_subsystem(elevator_subsystem), _testing_interface(testing_interface) {
    // Use addRequirements() here to delare subsystem dependencies.
    AddRequirements({_elevator_subsystem});
}

// Called when the command is initially scheduled.
void TestElevatorCommand::Initialize() {
    _elevator_subsystem->SetTestMode(true);
    frc::SmartDashboard::PutBoolean("Test Elevator", true);
}

// Called repeatedly when this Command is scheduled to run
void TestElevatorCommand::Execute() {
    if (frc::SmartDashboard::GetBoolean("Test Elevator", false)) {
        _elevator_subsystem->SetPower(_testing_interface->GetMotorOne());
        _elevator_subsystem->PrintTestInfo();
    }
}

// Called once the command ends or is interrupted.
void TestElevatorCommand::End(bool interrupted) {
    _elevator_subsystem->SetTestMode(false);
}

// Returns true when the command should end.
bool TestElevatorCommand::IsFinished() {
    return false;
}
