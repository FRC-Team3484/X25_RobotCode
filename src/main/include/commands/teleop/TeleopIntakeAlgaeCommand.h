// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#ifndef TELEOP_INTAKE_ALGAE_COMMAND_H
#define TELEOP_INTAKE_ALGAE_COMMAND_H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/IntakeSubsystem.h"
#include "subsystems/PivotSubsystem.h"
#include "subsystems/DrivetrainSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"

#include "OI.h"

/**
 * Intakes an algae, by raising the elevator, extending the pivot, and running the intake
 */
class TeleopIntakeAlgaeCommand
    : public frc2::CommandHelper<frc2::Command, TeleopIntakeAlgaeCommand>
    {
    
    public:
        explicit TeleopIntakeAlgaeCommand(DrivetrainSubsystem* drivetrain, ElevatorSubsystem* elevator, IntakeSubsystem* intake, PivotSubsystem* pivot, Operator_Interface* oi);

        void Initialize() override;

        void Execute() override;

        void End(bool interrupted) override;

        bool IsFinished() override;

    private:
        DrivetrainSubsystem* _drivetrain;
        ElevatorSubsystem* _elevator;
        IntakeSubsystem* _intake; 
        PivotSubsystem* _pivot;
        Operator_Interface* _oi;

        
        enum State {wait, extend_elevator, extend_pivot, intake, done};
        State _auto_intake_algae_state = wait;

};

#endif