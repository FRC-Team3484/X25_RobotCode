#include "commands/testing/TestPivotCommand.h"
#include "frc/smartdashboard/SmartDashboard.h"

#include <units/math.h>

TestPivotCommand::TestPivotCommand(PivotSubsystem *pivot_subsystem, Testing_Interface *testing_interface)
    : _pivot_subsystem(pivot_subsystem), _testing_interface(testing_interface) {
    AddRequirements({_pivot_subsystem});
}

void TestPivotCommand::Initialize() {
    _pivot_subsystem->SetTestMode(true);
    frc::SmartDashboard::PutBoolean("Test Pivot", false);
}

void TestPivotCommand::Execute() {
    _pivot_subsystem->PrintTestInfo();

    if (frc::SmartDashboard::GetBoolean("Test Pivot", false)) {
        if (_testing_interface->GetB()) {
            _pivot_subsystem->SetPivotAngle(PivotConstants::TARGET_CORAL_ANGLE);
        } else if (std::abs(_testing_interface->GetRawPivot()) > 0.0) {
            _pivot_subsystem->SetPower(_testing_interface->GetRawPivot());
        }
        else {
            _pivot_subsystem->SetPivotAngle(PivotConstants::HOME_POSITION);
        }
    }
}

void TestPivotCommand::End(bool interrupted) {
    _pivot_subsystem->SetTestMode(false);
}

bool TestPivotCommand::IsFinished() {
    return false; //(_testing_interface->GetB() && _pivot_subsystem->AtTargetPosition());
}
