#include "subsystems/LEDSubsystem.h"


LEDSubsystem::LEDSubsystem(
    int led_pwm_port,
    int led_strip_length
    ) :
    _leds(led_pwm_port)
    {
    _led_buffer.assign(led_strip_length, frc::AddressableLED::LEDData());       
}

void LEDSubsystem::Periodic() {
    switch (led_state) {
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
        led_state = idle;
        break;
    }
}

void LEDSubsystem::IdleAnimation() {
    
}
// scrolling rainbow

void LEDSubsystem::DrivingAnimation() {

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
// Driving But Green 

void LEDSubsystem::SetState(State state) {
    led_state = state;
}