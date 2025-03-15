#include "commands/testing/TestPivotCommand.h"
#include "frc/smartdashboard/SmartDashboard.h"

TestPivotCommand::TestPivotCommand(PivotSubsystem *pivot_subsystem, Testing_Interface *testing_interface)
    : _pivot_subsystem(pivot_subsystem), _testing_interface(testing_interface) {
    AddRequirements({_pivot_subsystem});
}

void TestPivotCommand::Initialize() {
    _pivot_subsystem->SetTestMode(true);
    frc::SmartDashboard::PutBoolean("Test Pivot", false);
}

void TestPivotCommand::Execute() {

    if (frc::SmartDashboard::GetBoolean("Test Pivot", false)) {
        _pivot_subsystem->SetPower(_testing_interface->GetRawPivot());
        /*if (_testing_interface->GetB()){
        _pivot_subsystem->SetPivotAngle(PivotConstants::TARGET_CORAL_ANGLE);
        } else {
            _pivot_subsystem->SetPivotAngle(PivotConstants::HOME_POSITION);
        }*/
    }
}

void TestPivotCommand::End(bool interrupted) {
    _pivot_subsystem->SetTestMode(false);
}

bool TestPivotCommand::IsFinished() {
    return false; //(_testing_interface->GetB() && _pivot_subsystem->AtTargetPosition());
}
