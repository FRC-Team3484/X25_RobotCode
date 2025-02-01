#ifndef LED_SUBSYSTEM_H
#define LED_SUBSYSTEM_H

#include <frc2/command/SubsystemBase.h>
#include <array>
#include <frc/AddressableLED.h>


class LEDSubsystem : public frc2::SubsystemBase {
    public:
        LEDSubsystem(
            int led_pwm_port,
            int led_strip_length
        );

        void Periodic() override;

        enum State {idle, driving, path, pivot, scoring, has_algae, has_coral};
        State led_state = idle;

        void SetState(State state);

        void IdleAnimation();

        void DrivingAnimation();

        void PathAnimation();

        void PivotAnimation();

        void ScoringAnimation();

        void HasAlgaeAnimation();

        void HasCoralAnimation();

    private:
        frc::AddressableLED _leds;
        std::vector<frc::AddressableLED::LEDData> _led_buffer;

};
#endif