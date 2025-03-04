#ifndef LED_SUBSYSTEM_H
#define LED_SUBSYSTEM_H

#include <frc2/command/SubsystemBase.h>
#include <array>
#include <frc/AddressableLED.h>
#include <frc/LEDPattern.h>
#include "ColorWave.h"
#include "FallingSand.h"
#include "Fire.h"
#include "ColorStack.h"
#include "Constants.h"
#include <frc/Timer.h>

class LEDSubsystem : public frc2::SubsystemBase {
    public:
        LEDSubsystem(
            int led_pwm_port,
            int led_strip_length
        );

        void Periodic() override;

        void WaveAnimation();

        void TetrisAnimation();

        void SandAnimation();

        void FireAnimation();

        void DrivingAnimation();

        void PathAnimation();

        void PivotAnimation();

        void ScoringAnimation();

        void HasAlgaeAnimation();

        void HasCoralAnimation();

        void StartIdleAnimation();

    private:

        enum State {wave, tetris, sand, fire, driving, path, pivot, scoring, has_algae, has_coral};
        State _led_state = tetris;

        frc::AddressableLED _leds;
        std::vector<frc::AddressableLED::LEDData> _led_buffer;

        ColorWave _colorwave{LEDConstants::COLORS, LEDConstants::LED_SPACING, LEDConstants::WAVELENGTH, LEDConstants::GAMMA, LEDConstants::SCROLLING_SPEED};
        ColorStack _tetris{LEDConstants::COLORS, LEDConstants::BAR_SIZE, LEDConstants::LED_SPACING, LEDConstants::VELOCITY, LEDConstants::FILL_SIZE, LEDConstants::EMPTY_SIZE, LEDConstants::GAMMA};
        FallingSand _sand{LEDConstants::COLORS, LEDConstants::BAR_SIZE, LEDConstants::LED_SPACING, LEDConstants::VELOCITY, LEDConstants::EXIT_ACCELERATION, LEDConstants::FILL_SIZE, LEDConstants::GAMMA};
        Fire _fire{LEDConstants::FIRE_HEIGHT, LEDConstants::SPARKS, LEDConstants::DELAY, LEDConstants::LED_STRIP_LENGTH};
        

        frc::LEDPattern _solid_orange;
        frc::LEDPattern _solid_green;
        frc::LEDPattern _solid_pink;
        frc::LEDPattern _solid_blue;

        frc::Timer _timer;
        std::array<std::pair<double, frc::Color>, 2> step_mask{std::pair{0.0, frc::Color::kWhite}, std::pair{0.3, frc::Color::kBlack}};
        frc::LEDPattern _scrolling_step = frc::LEDPattern::Steps(step_mask).ScrollAtAbsoluteSpeed(1.0_mps, LEDConstants::LED_SPACING);
        frc::LEDPattern _step_orange;

        frc::LEDPattern _progress_bar = frc::LEDPattern::ProgressMaskLayer([this]() { return _PivotAnimationProgress(); });
        double _PivotAnimationProgress(); 
        frc::LEDPattern _progress_orange;

        frc::LEDPattern _scoring_blue;


};
#endif