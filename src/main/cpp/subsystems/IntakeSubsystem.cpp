#include "subsystems/IntakeSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "FRC3484_Lib/utils/SC_Datatypes.h"

using namespace IntakeConstants;
using namespace ctre::phoenix6;

IntakeSubsystem::IntakeSubsystem(
    int _motor_can_id,
    int _algae_top_sensor_di_ch,
    int _algae_bottom_sensor_di_ch,
    int _coral_high_sensor_di_ch,
    int _coral_low_sensor_di_ch
    ) : 
        _intake_motor{_motor_can_id},
        _algae_top_sensor{_algae_top_sensor_di_ch},
        _algae_bottom_sensor{_algae_bottom_sensor_di_ch},
        _coral_high_sensor{_coral_high_sensor_di_ch},
        _coral_low_sensor{_coral_low_sensor_di_ch}
    {

    configs::TalonFXSConfiguration motor_config{};
    motor_config.MotorOutput.Inverted = INVERT_MOTOR;
    motor_config.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
    motor_config.Commutation.MotorArrangement = signals::MotorArrangementValue::Minion_JST;
    //motor_config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.05_s;

    // Load motor configs
    SC::SC_SwerveCurrents intake_current_constants;

    configs::CurrentLimitsConfigs steer_current_limit{};
    steer_current_limit
        .WithSupplyCurrentLimitEnable(intake_current_constants.Current_Limit_Enable)
        .WithSupplyCurrentLimit(intake_current_constants.Current_Limit_Steer)
        .WithSupplyCurrentLowerLimit(intake_current_constants.Steer_Current_Threshold)
        .WithSupplyCurrentLowerTime(intake_current_constants.Steer_Current_Time);

    motor_config.CurrentLimits = steer_current_limit;

    _intake_motor.GetConfigurator().Apply(motor_config);
};

void IntakeSubsystem::Periodic() {}

void IntakeSubsystem::SetPower(double power) {
    _intake_motor.Set(power);
}

bool IntakeSubsystem::HasAlgae() {
    return !_algae_top_sensor.Get() && !_algae_bottom_sensor.Get();
}

bool IntakeSubsystem::CoralHigh() {
    return !_coral_high_sensor.Get();
}

bool IntakeSubsystem::CoralLow() {
    return !_coral_low_sensor.Get();
}

bool IntakeSubsystem::HasCoral() {
    return CoralHigh() || CoralLow();
}

void IntakeSubsystem::PrintTestInfo() {
    frc::SmartDashboard::PutBoolean("Has Algae", HasAlgae());
    frc::SmartDashboard::PutBoolean("Has Algae 1", !_algae_top_sensor.Get());
    frc::SmartDashboard::PutBoolean("Has Algae 2", !_algae_bottom_sensor.Get());
    
    frc::SmartDashboard::PutBoolean("Have Coral High", CoralHigh());
    frc::SmartDashboard::PutBoolean("Have Coral Low", CoralLow());
}