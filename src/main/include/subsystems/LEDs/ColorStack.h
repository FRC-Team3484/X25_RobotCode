#ifndef COLORSTACK_H
#define COLORSTACK_H

#include <vector>

#include <units/math.h>
#include <units/length.h>
#include <units/velocity.h>

#include <frc/LEDPattern.h>
#include <frc/AddressableLED.h>

class ColorStack {
    public:
        ColorStack(std::vector<frc::Color> colors, int bar_size, units::meter_t led_spacing, units::meters_per_second_t velocity, size_t fill_size, size_t empty_size, double gamma);

        void ApplyTo(std::span<frc::AddressableLED::LEDData> data);

        void Reset();

        bool HasResetted();

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
        double _velocity;
        size_t _fill_size;
        size_t _empty_size;
        double _gamma;

        size_t _leds_placed;
        double _falling_led_position;

};

#endif