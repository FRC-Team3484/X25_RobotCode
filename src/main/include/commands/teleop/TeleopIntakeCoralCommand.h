// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#ifndef TELEOP_INTAKE_CORAL_COMMAND_H
#define TELEOP_INTAKE_CORAL_COMMAND_H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/IntakeSubsystem.h"
#include "subsystems/PivotSubsystem.h"
#include "subsystems/DrivetrainSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"
#include "subsystems/FunnelSubsystem.h"

#include "OI.h"

class TeleopIntakeCoralCommand
    : public frc2::CommandHelper<frc2::Command, TeleopIntakeCoralCommand>
    {
    
    public:
        /**
         * Intakes a coral by raising the elevator, extending the pivot, and running the intake
         * 
         * @param drivetrain A pointer to the drivetrain subsystem
         * @param elevator A pointer to the elevator subsystem
         * @param intake A pointer to the intake subsystem
         * @param pivot A pointer to the pivot subsystem
         * @param oi A pointer to the operator interface
         */
        explicit TeleopIntakeCoralCommand(
            DrivetrainSubsystem* drivetrain, 
            ElevatorSubsystem* elevator, 
            IntakeSubsystem* intake, 
            PivotSubsystem* pivot, 
            FunnelSubsystem* funnel, 
            Operator_Interface* oi
        );

        void Initialize() override;

        void Execute() override;

        void End(bool interrupted) override;

        bool IsFinished() override;

    private:
        DrivetrainSubsystem* _drivetrain;
        ElevatorSubsystem* _elevator;
        IntakeSubsystem* _intake; 
        PivotSubsystem* _pivot;
        FunnelSubsystem* _funnel;
        Operator_Interface* _oi;

        enum State {wait, ready_intake, intake, done};
        State _auto_intake_coral_state = wait;

};

#endif