#include "subsystems/LEDSubsystem.h"
#include "FRC3484_Lib/utils/SC_Functions.h"


LEDSubsystem::LEDSubsystem(
    int led_pwm_port,
    int led_strip_length
    ) :
    _leds(led_pwm_port),
    _solid_orange{frc::LEDPattern::Solid(SC_GammaCorrection(LEDConstants::DRIVE_ORANGE, LEDConstants::GAMMA))},
    _solid_green{frc::LEDPattern::Solid(SC_GammaCorrection(LEDConstants::ALGAE_GREEN, LEDConstants::GAMMA))},
    _solid_pink{frc::LEDPattern::Solid(SC_GammaCorrection(LEDConstants::CORAL_PINK, LEDConstants::GAMMA))},
    _solid_blue{frc::LEDPattern::Solid(SC_GammaCorrection(LEDConstants::TEAM_BLUE, LEDConstants::GAMMA))},
    _step_orange{_solid_orange.Mask(_scrolling_step)},
    _progress_orange{_solid_orange.Mask(_progress_bar)},
    _scoring_blue{_solid_blue.Blink(0.2_s)}
    {
    _led_buffer.assign(led_strip_length, frc::AddressableLED::LEDData()); 
    _leds.SetLength(led_strip_length);
    _leds.SetData(_led_buffer);
    _leds.Start();
    _timer.Start();
}



void LEDSubsystem::Periodic() {
    switch (_led_state) {
    case idle:
        IdleAnimation();
        break;
    case driving:
        DrivingAnimation();
        break;
    case path:
        PathAnimation();
        break;
    case pivot:
        PivotAnimation();
        break;
    case scoring:
        ScoringAnimation();
        break;
    case has_algae:
        HasAlgaeAnimation();
        break;
    case has_coral:
        HasCoralAnimation();
        break;

    default:
        _led_state = idle;
        break;
    }
    _leds.SetData(_led_buffer);
}

void LEDSubsystem::IdleAnimation() {
    _colorwave.ApplyTo(_led_buffer);
}
// scrolling custom rainbow

void LEDSubsystem::DrivingAnimation() {
    _solid_orange.ApplyTo(_led_buffer);
}
// solid color (Orange #ff8200)

void LEDSubsystem::PathAnimation() {
    _step_orange.ApplyTo(_led_buffer);
}
// panning mask

void LEDSubsystem::PivotAnimation() {
    _progress_orange.ApplyTo(_led_buffer);
}
// mask

void LEDSubsystem::ScoringAnimation() {
    _scoring_blue.ApplyTo(_led_buffer);
}
// Breathe (Blinks Blue #009bb4)

void LEDSubsystem::HasAlgaeAnimation() {
    _solid_green.ApplyTo(_led_buffer);
}
// Driving But Green #10f01a

void LEDSubsystem::HasCoralAnimation() {
    _solid_pink.ApplyTo(_led_buffer);
}
// Driving But Pink #ff0091

void LEDSubsystem::StartIdleAnimation() {
    _led_state = idle;
}   

double LEDSubsystem::_PivotAnimationProgress() {
    return std::fmod((_timer.Get() / LEDConstants::PIVOT_ANIMATION_TIME).to<double>(), 1.0);
}