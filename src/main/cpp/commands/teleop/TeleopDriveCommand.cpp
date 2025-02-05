
#include "commands/teleop/TeleopDriveCommand.h"

#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <units/angle.h>
#include <frc/kinematics/SwerveModuleState.h>

using namespace units;
using namespace frc;
using namespace SwerveConstants::AutonDriveConstants;
using namespace SwerveConstants::BrakeConstants;
using namespace SwerveConstants::DrivetrainConstants::JoystickScaling;

TeleopDriveCommand::TeleopDriveCommand(DrivetrainSubsystem* drivetrain, Driver_Interface* oi) 
    : _drivetrain{drivetrain}, _oi{oi} {
    AddRequirements(_drivetrain);
}

void TeleopDriveCommand::Initialize() {
    _drivetrain->SetBrakeMode();

    alliance = frc::DriverStation::GetAlliance();
    if (!alliance.has_value()) {
        fmt::print("Error: Teleop Drive Command failed to determine alliance station");
        alliance = DriverStation::Alliance::kBlue;
    }
}

void TeleopDriveCommand::Execute() {
    // Repeated Data Grabbers
    wpi::array<SwerveModulePosition, 4> current_positions = _drivetrain->GetModulePositions();

    // Logic with buttons
    if (_oi != NULL) {

        switch(_drivetrain_state){
            case drive:
                if (_oi->GetResetHeading()) {
                    _drivetrain->SetHeading();
                }

                if (_oi->GetSetBrakeMode()) {
                    _drivetrain->SetBrakeMode();
                }

                if (_oi->GetDisableBrakeMode()) {
                    _drivetrain->SetCoastMode();
                }

                if (_oi->GetDynamicPivot()){
                    _drivetrain_state = pivot;

                    // Created objects
                    _pivot_corner = {1_m, copysign(1.0, _oi->GetRotation())*1_m};
                    _pivot_drive = {-(_oi->GetThrottle())*1_m, -(_oi->GetStrafe())*1_m};

                    _pivot_corner.RotateBy(_pivot_drive.Angle());
                    _pivot_corner.RotateBy(_drivetrain->GetPose().Rotation());

                    _pivot_corner = {copysign(1.0, _pivot_corner.X().value()) * SwerveConstants::DrivetrainConstants::DRIVETRAIN_LENGTH, copysign(1.0, _pivot_corner.Y().value()) * SwerveConstants::DrivetrainConstants::DRIVETRAIN_WIDTH};
                }

                if (_oi->GetBrake()) {
                    _drivetrain->SetModuleStates(
                        {
                        SwerveModuleState{(_initial_positions[FL].distance - current_positions[FL].distance) * DYNAMIC_BRAKE_SCALING * MAX_LINEAR_SPEED, 45_deg},
                        SwerveModuleState{(_initial_positions[FR].distance - current_positions[FR].distance) * DYNAMIC_BRAKE_SCALING * MAX_LINEAR_SPEED, -45_deg},
                        SwerveModuleState{(_initial_positions[BL].distance - current_positions[BL].distance) * DYNAMIC_BRAKE_SCALING * MAX_LINEAR_SPEED, -45_deg},
                        SwerveModuleState{(_initial_positions[BR].distance - current_positions[BR].distance) * DYNAMIC_BRAKE_SCALING * MAX_LINEAR_SPEED, 45_deg}
                        },
                        true,
                        false
                    );
                    
                } else {
                    // Logic for actual joystick movements

                    meters_per_second_t x_speed = -_oi->GetThrottle() * MAX_LINEAR_SPEED * (alliance.value() == DriverStation::Alliance::kRed ? -1 : 1);
                    meters_per_second_t y_speed = -_oi->GetStrafe() * MAX_LINEAR_SPEED * (alliance.value() == DriverStation::Alliance::kRed ? -1 : 1);
                    radians_per_second_t rotation = -_oi->GetRotation() * MAX_ROTATION_SPEED;

                    if (_oi->LowSpeed()) {
                        x_speed *= LOW_SCALE;
                        y_speed *= LOW_SCALE;
                        rotation *= LOW_SCALE;
                    }
                    
                    _drivetrain->Drive(x_speed, y_speed, rotation, true);
                }
                break;
            case pivot:
                
                break;
            default:
                _drivetrain_state=drive;
                break;
        }
        
    }
}

void TeleopDriveCommand::End(bool interrupted) {
    _drivetrain->StopMotors();
}

bool TeleopDriveCommand::IsFinished() {return false;}