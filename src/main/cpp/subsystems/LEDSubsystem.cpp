#include "subsystems/LEDSubsystem.h"


LEDSubsystem::LEDSubsystem(
    int led_pwm_port,
    int led_strip_length
    ) :
    _leds(led_pwm_port)
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

}
// panning mask

void LEDSubsystem::PivotAnimation() {

}
// mask

void LEDSubsystem::ScoringAnimation() {

}
// Breathe (Blinks Blue #009bb4)

void LEDSubsystem::HasAlgaeAnimation() {

}
// Driving But Pink #ff0091

void LEDSubsystem::HasCoralAnimation() {

}
// Driving But Green #7fffd4

void LEDSubsystem::StartIdleAnimation() {
    _led_state = idle;
}