#include "commands/testing/TestIntakeCommand.h"
#include <frc/smartdashboard/SmartDashboard.h>

TestIntakeCommand::TestIntakeCommand(IntakeSubsystem *intake_subsystem, Testing_Interface *testing_interface)
    : _intake_subsystem{intake_subsystem}, _testing_interface{testing_interface} {
    AddRequirements(_intake_subsystem);
}

void TestIntakeCommand::Initialize() {
    frc::SmartDashboard::PutBoolean("Test Intake", false);
}

void TestIntakeCommand::Execute() {
    _intake_subsystem->PrintTestInfo();

    if (frc::SmartDashboard::GetBoolean("Test Intake", false)) {
        _intake_subsystem->SetPower(_testing_interface->GetRawIntake());
    }
}

void TestIntakeCommand::End(bool interrupted) {}

bool TestIntakeCommand::IsFinished() {
    return false;
}
