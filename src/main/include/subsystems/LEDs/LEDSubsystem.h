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

        void LowBatteryAnimation();

        void DrivingAnimation();

        void PathAnimation();

        void PivotAnimation();

        void ScoringAnimation();

        void ClimbAnimation();

        void ElevatorHomingAnimation();

        void HasAlgaeAnimation();

        void HasCoralAnimation();

    private:

        frc::AddressableLED _leds;
        std::vector<frc::AddressableLED::LEDData> _led_buffer;
        std::span<frc::AddressableLED::LEDData> _bottom_leds;
        std::span<frc::AddressableLED::LEDData> _top_leds;

        ColorWave _colorwave{LEDConstants::COLORS, LEDConstants::LED_SPACING, LEDConstants::WAVELENGTH, LEDConstants::GAMMA, LEDConstants::SCROLLING_SPEED};
        ColorStack _tetris_bottom{LEDConstants::COLORS, LEDConstants::BAR_SIZE, LEDConstants::LED_SPACING, LEDConstants::VELOCITY, LEDConstants::FILL_SIZE, LEDConstants::EMPTY_SIZE, LEDConstants::GAMMA};
        ColorStack _tetris_top{LEDConstants::COLORS, LEDConstants::BAR_SIZE, LEDConstants::LED_SPACING, LEDConstants::VELOCITY, LEDConstants::FILL_SIZE, LEDConstants::EMPTY_SIZE, LEDConstants::GAMMA};
        FallingSand _sand_bottom{LEDConstants::COLORS, LEDConstants::BAR_SIZE, LEDConstants::LED_SPACING, LEDConstants::VELOCITY, LEDConstants::EXIT_ACCELERATION, LEDConstants::FILL_SIZE, LEDConstants::GAMMA};
        FallingSand _sand_top{LEDConstants::COLORS, LEDConstants::BAR_SIZE, LEDConstants::LED_SPACING, LEDConstants::VELOCITY, LEDConstants::EXIT_ACCELERATION, LEDConstants::FILL_SIZE, LEDConstants::GAMMA};
        Fire _fire{LEDConstants::FIRE_HEIGHT, LEDConstants::SPARKS, LEDConstants::DELAY, LEDConstants::LED_STRIP_LENGTH};
        

        frc::LEDPattern _solid_orange;
        frc::LEDPattern _solid_green;
        frc::LEDPattern _solid_pink;
        frc::LEDPattern _solid_blue;
        frc::LEDPattern _solid_red;

        frc::Timer _timer;
        std::array<std::pair<double, frc::Color>, 2> step_mask{std::pair{0.0, frc::Color::kWhite}, std::pair{0.3, frc::Color::kBlack}};
        frc::LEDPattern _scrolling_step = frc::LEDPattern::Steps(step_mask).ScrollAtAbsoluteSpeed(1.0_mps, LEDConstants::LED_SPACING);
        frc::LEDPattern _step_orange;

        frc::LEDPattern _progress_bar = frc::LEDPattern::ProgressMaskLayer([this]() { return _PivotAnimationProgress(); });
        double _PivotAnimationProgress(); 
        frc::LEDPattern _progress_orange;

        std::array<std::pair<double, frc::Color>, 2> first_level{std::pair{0.0, frc::Color::kBlack}, std::pair{0.4, frc::Color::kWhite}, std::pair{0.6, frc::Color::kBlack}};
        frc::LEDPattern _score_one_mask = frc::LEDPattern::Mask(first_level);
        frc::LEDPattern _scoring_one_blue;

        std::array<std::pair<double, frc::Color>, 2> second_level{std::pair{0.0, frc::Color::kBlack}, std::pair{0.2, frc::Color::kWhite}, std::pair{0.4, frc::Color::kBlack}, std::pair{0.6, frc::Color::kWhite}, std::pair{0.8, frc::Color::kBlack},};
        frc::LEDPattern _score_two_mask;
        frc::LEDPattern _scoring_two_blue;

        std::array<std::pair<double, frc::Color>, 2> third_level{std::pair{0.0, frc::Color::kWhite}, std::pair{0.2, frc::Color::kBlack}, std::pair{0.4, frc::Color::kWhite}, std::pair{0.6, frc::Color::kBlack}, std::pair{0.8, frc::Color::kWhite},};
        frc::LEDPattern _score_three_mask;
        frc::LEDPattern _scoring_three_blue;

        frc::LEDPattern _scoring_four_blue;

        frc::LEDPattern _low_battery;

        frc::LEDPattern _elevator_home;

        frc::LEDPattern _climb_mask;


};
#endif