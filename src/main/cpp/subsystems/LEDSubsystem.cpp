#include "subsystems/LEDSubsystem.h"
#include "FRC3484_Lib/utils/SC_Functions.h"


LEDSubsystem::LEDSubsystem(
    int led_pwm_port,
    int led_strip_length
    ) :
    _leds(led_pwm_port),
    _solid_orange{LEDPattern::Solid(SC_GammaCorrection(LEDConstants::DRIVE_ORANGE, LEDConstants::GAMMA))},
    _solid_green{LEDPattern::Solid(SC_GammaCorrection(LEDConstants::ALGAE_GREEN, LEDConstants::GAMMA))},
    _solid_pink{LEDPattern::Solid(SC_GammaCorrection(LEDConstants::CORAL_PINK, LEDConstants::GAMMA))},
    _step_orange{_solid_orange.Mask(_scrolling_step)}
    //_progress_bar{_solid_orange.Mask(ProgressMaskLayer)}
    {
    _led_buffer.assign(led_strip_length, frc::AddressableLED::LEDData()); 
    _leds.SetLength(led_strip_length);
    _leds.SetData(_led_buffer);
    _leds.Start();      
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

}
// mask

void LEDSubsystem::ScoringAnimation() {

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