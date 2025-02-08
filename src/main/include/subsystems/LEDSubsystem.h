#ifndef LED_SUBSYSTEM_H
#define LED_SUBSYSTEM_H

#include <frc2/command/SubsystemBase.h>
#include <array>
#include <frc/AddressableLED.h>
#include <frc/LEDPattern.h>
#include "subsystems/ColorWave.h"
#include "Constants.h"
#include <frc/Timer.h>

using namespace frc; 

class LEDSubsystem : public frc2::SubsystemBase {
    public:
        LEDSubsystem(
            int led_pwm_port,
            int led_strip_length
        );

        void Periodic() override;

        void IdleAnimation();

        void DrivingAnimation();

        void PathAnimation();

        void PivotAnimation();

        void ScoringAnimation();

        void HasAlgaeAnimation();

        void HasCoralAnimation();

        void StartIdleAnimation();

    private:

        enum State {idle, driving, path, pivot, scoring, has_algae, has_coral};
        State _led_state = idle;

        AddressableLED _leds;
        std::vector<AddressableLED::LEDData> _led_buffer;

        ColorWave _colorwave{LEDConstants::COLORS, LEDConstants::LED_SPACING, LEDConstants::WAVELENGTH, LEDConstants::GAMMA, LEDConstants::SCROLLING_SPEED};

        LEDPattern _solid_orange;
        LEDPattern _solid_green;
        LEDPattern _solid_pink;

        std::array<std::pair<double, Color>, 2> step_mask{std::pair{0.0, Color::kWhite}, std::pair{0.5, Color::kBlack}};
        LEDPattern _scrolling_step = LEDPattern::Steps(step_mask).ScrollAtAbsoluteSpeed(LEDConstants::SCROLLING_SPEED, LEDConstants::LED_SPACING);
        LEDPattern _step_orange;

        //frc::Timer _timer;
        //LEDPattern _progress_bar = LEDPattern::ProgressMaskLayer({});





};
#endif