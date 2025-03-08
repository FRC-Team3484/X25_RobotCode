#include "SysIdRoutineBot.h"


#include <frc2/command/Commands.h>

SysIdRoutineBot::SysIdRoutineBot(DrivetrainSubsystem* drive, ElevatorSubsystem* elevator, PivotSubsystem* pivot) : 
    _drive{drive},
    _elevator{elevator},
    _pivot{pivot}
    {

    _elevator->SetDefaultCommand(_elevator->PseudoMoveCommand([this] { return _sysid_driver_testing_controller.GetLeftY() * _sysid_driver_testing_controller.GetLeftBumperButton() * !_sysid_driver_testing_controller.GetRightBumperButton(); }));
    frc2::Trigger([this]{return (_sysid_driver_testing_controller.GetAButton() && _sysid_driver_testing_controller.GetLeftBumperButton()) && !_sysid_driver_testing_controller.GetRightBumperButton();})
        .WhileTrue(_elevator->SysIdQuasistatic(frc2::sysid::Direction::kForward));
    frc2::Trigger([this]{return (_sysid_driver_testing_controller.GetBButton() && _sysid_driver_testing_controller.GetLeftBumperButton()) && !_sysid_driver_testing_controller.GetRightBumperButton();})
        .WhileTrue(_elevator->SysIdQuasistatic(frc2::sysid::Direction::kReverse));
    frc2::Trigger([this]{return (_sysid_driver_testing_controller.GetXButton() && _sysid_driver_testing_controller.GetLeftBumperButton()) && !_sysid_driver_testing_controller.GetRightBumperButton();})
        .WhileTrue(_elevator->SysIdDynamic(frc2::sysid::Direction::kForward));
    frc2::Trigger([this]{return (_sysid_driver_testing_controller.GetYButton() && _sysid_driver_testing_controller.GetLeftBumperButton()) && !_sysid_driver_testing_controller.GetRightBumperButton();})
        .WhileTrue(_elevator->SysIdDynamic(frc2::sysid::Direction::kReverse));
    
    _pivot->SetDefaultCommand(_pivot->PseudoMoveCommand([this] { return _sysid_driver_testing_controller.GetLeftY() * _sysid_driver_testing_controller.GetRightBumperButton() *!_sysid_driver_testing_controller.GetLeftBumperButton(); }));
    frc2::Trigger([this]{return (_sysid_driver_testing_controller.GetAButton() && _sysid_driver_testing_controller.GetRightBumperButton());})
        .WhileTrue(_pivot->SysIdQuasistatic(frc2::sysid::Direction::kForward));
    frc2::Trigger([this]{return (_sysid_driver_testing_controller.GetBButton() && _sysid_driver_testing_controller.GetRightBumperButton());})
        .WhileTrue(_pivot->SysIdQuasistatic(frc2::sysid::Direction::kReverse));
    frc2::Trigger([this]{return (_sysid_driver_testing_controller.GetXButton() && _sysid_driver_testing_controller.GetRightBumperButton());})
        .WhileTrue(_pivot->SysIdDynamic(frc2::sysid::Direction::kForward));
    frc2::Trigger([this]{return (_sysid_driver_testing_controller.GetYButton() && _sysid_driver_testing_controller.GetRightBumperButton());})
        .WhileTrue(_pivot->SysIdDynamic(frc2::sysid::Direction::kReverse));

    _drive->SetDefaultCommand(_drive->PseudoDriveCommand([this] { return _sysid_driver_testing_controller.GetLeftY() * !_sysid_driver_testing_controller.GetLeftBumperButton()* !_sysid_driver_testing_controller.GetRightBumperButton(); },
                                                         [this] { return _sysid_driver_testing_controller.GetLeftX() * !_sysid_driver_testing_controller.GetLeftBumperButton()* !_sysid_driver_testing_controller.GetRightBumperButton(); },
                                                         [this] { return _sysid_driver_testing_controller.GetRightX() * !_sysid_driver_testing_controller.GetLeftBumperButton()* !_sysid_driver_testing_controller.GetRightBumperButton(); }));
    frc2::Trigger([this]{return (_sysid_driver_testing_controller.GetAButton() && !_sysid_driver_testing_controller.GetLeftBumperButton()) && !_sysid_driver_testing_controller.GetRightBumperButton();})
        .WhileTrue(_drive->SysIdQuasistatic(frc2::sysid::Direction::kForward, _sysid_driver_testing_controller.GetLeftTriggerAxis() > 0.5 ? 90_deg : 0_deg));
    frc2::Trigger([this]{return (_sysid_driver_testing_controller.GetBButton() && !_sysid_driver_testing_controller.GetLeftBumperButton()) && !_sysid_driver_testing_controller.GetRightBumperButton();})
        .WhileTrue(_drive->SysIdQuasistatic(frc2::sysid::Direction::kReverse, _sysid_driver_testing_controller.GetLeftTriggerAxis() > 0.5 ? 90_deg : 0_deg));
    frc2::Trigger([this]{return (_sysid_driver_testing_controller.GetXButton() && !_sysid_driver_testing_controller.GetLeftBumperButton()) && !_sysid_driver_testing_controller.GetRightBumperButton();})
        .WhileTrue(_drive->SysIdDynamic(frc2::sysid::Direction::kForward, _sysid_driver_testing_controller.GetLeftTriggerAxis() > 0.5 ? 90_deg : 0_deg));
    frc2::Trigger([this]{return (_sysid_driver_testing_controller.GetYButton() && !_sysid_driver_testing_controller.GetLeftBumperButton()) && !_sysid_driver_testing_controller.GetRightBumperButton();})       
        .WhileTrue(_drive->SysIdDynamic(frc2::sysid::Direction::kReverse, _sysid_driver_testing_controller.GetLeftTriggerAxis() > 0.5 ? 90_deg : 0_deg));

}