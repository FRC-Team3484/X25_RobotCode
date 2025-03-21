#include "subsystems/LEDs/LEDSubsystem.h"
#include "FRC3484_Lib/utils/SC_Functions.h"

#include <ranges>


LEDSubsystem::LEDSubsystem(
    int led_pwm_port,
    int led_strip_length
    ) :
    _leds(led_pwm_port),
    _solid_orange{frc::LEDPattern::Solid(SC_GammaCorrection(LEDConstants::DRIVE_ORANGE, LEDConstants::GAMMA))},
    _solid_green{frc::LEDPattern::Solid(SC_GammaCorrection(LEDConstants::ALGAE_GREEN, LEDConstants::GAMMA))},
    _solid_pink{frc::LEDPattern::Solid(SC_GammaCorrection(LEDConstants::CORAL_PINK, LEDConstants::GAMMA))},
    _solid_blue{frc::LEDPattern::Solid(SC_GammaCorrection(LEDConstants::TEAM_BLUE, LEDConstants::GAMMA))},
    _solid_red{frc::LEDPattern::Solid(SC_GammaCorrection(LEDConstants::FIRE_RED, LEDConstants::GAMMA))},
    _step_orange{_solid_orange.Mask(_scrolling_step)},
    _progress_orange{_solid_orange.Mask(_progress_bar)},
    _scoring_blue{_solid_blue.Blink(LEDConstants::SCORING_BLUE_ON_TIME, LEDConstants::SCORING_BLUE_OFF_TIME)},
    _low_battery{_solid_red.Breathe(LEDConstants::LOW_BATTERY_CYCLE_TIME)}
    {
    _led_buffer.assign(led_strip_length, frc::AddressableLED::LEDData());
    _bottom_leds = std::span<frc::AddressableLED::LEDData>(_led_buffer).first(led_strip_length / 2);
    _top_leds = std::span<frc::AddressableLED::LEDData>(_led_buffer).last(led_strip_length / 2);
    _leds.SetLength(led_strip_length);
    _leds.SetData(_led_buffer);
    _leds.Start();
    _timer.Start();
}



void LEDSubsystem::Periodic() {
    
}

// scrolling custom rainbow
void LEDSubsystem::WaveAnimation() {
    _colorwave.ApplyTo(_bottom_leds);
    _colorwave.ApplyTo(_top_leds);
    std::reverse(_top_leds.begin(), _top_leds.end());
    _leds.SetData(_led_buffer);
}

// colors stacking idle
void LEDSubsystem::TetrisAnimation() {
    _tetris_bottom.ApplyTo(_bottom_leds);
    _tetris_top.ApplyTo(_top_leds);
    std::reverse(_top_leds.begin(), _top_leds.end());
    _leds.SetData(_led_buffer);
}

// colors falling idle
void LEDSubsystem::SandAnimation() {
    _sand_bottom.ApplyTo(_bottom_leds);
    _sand_top.ApplyTo(_top_leds);
    std::reverse(_top_leds.begin(), _top_leds.end());
    _leds.SetData(_led_buffer);
}

void LEDSubsystem::LowBatteryAnimation() {
    _low_battery.ApplyTo(_bottom_leds);
    _low_battery.ApplyTo(_top_leds);
    std::reverse(_top_leds.begin(), _top_leds.end());
    _leds.SetData(_led_buffer);
}

// solid color (Orange #ff8200)
void LEDSubsystem::DrivingAnimation() {
    _solid_orange.ApplyTo(_led_buffer);
    _leds.SetData(_led_buffer);
}


// panning mask
void LEDSubsystem::PathAnimation() {
    _step_orange.ApplyTo(_bottom_leds);
    _step_orange.ApplyTo(_top_leds);
    std::reverse(_top_leds.begin(), _top_leds.end());
    _leds.SetData(_led_buffer);
}

// mask
void LEDSubsystem::PivotAnimation() {
    _progress_orange.ApplyTo(_bottom_leds);
    _progress_orange.ApplyTo(_top_leds);
    std::reverse(_top_leds.begin(), _top_leds.end());
    _leds.SetData(_led_buffer);
}

// Breathe (Blinks Blue #009bb4)
void LEDSubsystem::ScoringAnimation() {
    _scoring_blue.ApplyTo(_led_buffer);
    _leds.SetData(_led_buffer);
}

// Driving But Green #10f01a
void LEDSubsystem::HasAlgaeAnimation() {
    _solid_green.ApplyTo(_led_buffer);
    _leds.SetData(_led_buffer);
}

// Driving But Pink #ff0091
void LEDSubsystem::HasCoralAnimation() {
    _solid_pink.ApplyTo(_led_buffer);
    _leds.SetData(_led_buffer);
}   

double LEDSubsystem::_PivotAnimationProgress() {
    return std::fmod((_timer.Get() / LEDConstants::PIVOT_ANIMATION_TIME).to<double>(), 1.0);
}