// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ElevatorSubsystem.h"

ElevatorSubsystem::ElevatorSubsystem(
    int primary_motor_can_id,
    int secondary_motor_can_id,
    int home_sensor_di_ch,
    SC::SC_PIDConstants elevator_pidc,
    units::feet_per_second_t max_velocity,
    units::feet_per_second_squared_t max_acceleration
    ) : 
    _primary_motor(primary_motor_can_id),
    _secondary_motor(secondary_motor_can_id),
    _home_sensor(home_sensor_di_ch),
    _elevator_pid_controller{elevator_pidc.Kp, elevator_pidc.Ki, elevator_pidc.Kd},
    _elevator_trapezoid{{max_velocity, max_acceleration}}
    {
        
}

// This method will be called once per scheduler run
void ElevatorSubsystem::Periodic() {
    
}
