#ifndef FALLINGSAND_H
#define FALLINGSAND_H

#include <vector>

#include <units/math.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/acceleration.h>

#include <frc/LEDPattern.h>
#include <frc/AddressableLED.h>

class FallingSand {
    public:
        FallingSand(std::vector<frc::Color> colors, int bar_size, units::meter_t led_spacing, units::meters_per_second_t intake_velocity, units::meters_per_second_squared_t exit_acceleration, size_t fill_size, double gamma);

        void ApplyTo(std::span<frc::AddressableLED::LEDData> data);
        void Reset();


    private:
        size_t _GetColorIndex(size_t offset);
        double _PositiveFmod(double numerator, double denominator);
        frc::Color _CorrectGamma(frc::Color color);

        enum PatternState {
            fill,
            empty
        };
        PatternState _state = fill;

        std::vector<frc::Color> _colors;
        size_t _bar_size;
        double _intake_velocity;
        double _exit_velocity;
        double _exit_acceleration;
        size_t _fill_size;
        size_t _empty_size;
        double _gamma;

        size_t _leds_placed;
        double _falling_led_position;
};

#endif