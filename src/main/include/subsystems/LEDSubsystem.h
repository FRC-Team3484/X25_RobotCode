#ifndef LED_SUBSYSTEM_H
#define LED_SUBSYSTEM_H

#include <frc2/command/SubsystemBase.h>
#include <array>
#include <frc/AddressableLED.h>
#include "subsystems/ColorWave.h"
#include "Constants.h"


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

        frc::AddressableLED _leds;
        std::vector<frc::AddressableLED::LEDData> _led_buffer;

        ColorWave _colorwave{LEDConstants::COLORS};



};
#endif