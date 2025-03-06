#include "SysIdRoutineBot.h"


#include <frc2/command/Commands.h>

SysIdRoutineBot::SysIdRoutineBot(DrivetrainSubsystem* drive, ElevatorSubsystem* elevator, PivotSubsystem* pivot, frc::EventLoop* event_loop) : 
    _drive{drive}, 
    _elevator{elevator},
    _pivot{pivot},
    _event_loop{event_loop}
    {

    _elevator->SetDefaultCommand(_elevator->PseudoMoveCommand([this] { return _sysid_driver_testing_controller.GetLeftY(); }));
    frc2::Trigger([this]{return (_sysid_driver_testing_controller.GetAButton() && _sysid_driver_testing_controller.GetLeftBumperButton());})
        .WhileTrue(_elevator->SysIdQuasistatic(frc2::sysid::Direction::kForward));
    frc2::Trigger([this]{return (_sysid_driver_testing_controller.GetBButton() && _sysid_driver_testing_controller.GetLeftBumperButton());})
        .WhileTrue(_elevator->SysIdQuasistatic(frc2::sysid::Direction::kReverse));
    frc2::Trigger([this]{return (_sysid_driver_testing_controller.GetXButton() && _sysid_driver_testing_controller.GetLeftBumperButton());})
        .WhileTrue(_elevator->SysIdDynamic(frc2::sysid::Direction::kForward));
    frc2::Trigger([this]{return (_sysid_driver_testing_controller.GetYButton() && _sysid_driver_testing_controller.GetLeftBumperButton());})
        .WhileTrue(_elevator->SysIdDynamic(frc2::sysid::Direction::kReverse));

}