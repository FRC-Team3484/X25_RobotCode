#include  "subsystems/LEDs/FallingSand.h"

FallingSand::FallingSand(std::vector<frc::Color> colors, int bar_size, units::meter_t led_spacing, units::meters_per_second_t intake_velocity, units::meters_per_second_squared_t exit_acceleration, size_t fill_size, double gamma) {
    _colors = colors;
    _bar_size = bar_size;
    _intake_velocity = intake_velocity.value();
    _exit_acceleration = exit_acceleration.value();
    _exit_velocity = intake_velocity.value() + exit_acceleration.value() * 0.5;
    _fill_size = fill_size;
    _empty_size = 0;
    _gamma = gamma;
    for (frc::Color color : colors) {
        _colors.push_back(_CorrectGamma(color));
    }
    Reset();
}

void FallingSand::Reset() {
    _leds_placed = 0;
    _falling_led_position = 0;
    _state = fill;
}

void FallingSand::ApplyTo(std::span<frc::AddressableLED::LEDData> data) {
    switch(_state) {
        case fill:
            _falling_led_position += _intake_velocity;
            if (_falling_led_position >= data.size() - _leds_placed) {
                _leds_placed = std::min(_leds_placed + _fill_size, data.size());
                _falling_led_position = _PositiveFmod(_falling_led_position, double(_fill_size));
            }

            for (size_t i = 0; i < data.size(); i++) {
                if (i >= data.size() - _leds_placed) {
                    data[i].SetLED(_colors[_GetColorIndex(i)]);
                } else if (i <= size_t(_falling_led_position) && i > size_t(_falling_led_position) - _fill_size) {
                    data[i].SetLED(_colors[_GetColorIndex(data.size() + i - _leds_placed - size_t(_falling_led_position) - 1)]);
                } else {
                    data[i].SetLED(frc::Color::kBlack);
                }
            }

            if (_leds_placed >= data.size()) {
                _state = empty;
                _falling_led_position = 0;
                _exit_velocity = 0;
            }

            break;
        
        case empty:
            _falling_led_position += _exit_velocity;
            _exit_velocity += _exit_acceleration;

            for(size_t i= 0; i < data.size(); i++){
                if (i<size_t(_falling_led_position)){
                    data[i].SetLED(frc::Color::kBlack);
                } else {
                    data[i].SetLED(_colors[_GetColorIndex(i - size_t(_falling_led_position))]);
                }
            }
            if(_falling_led_position >= data.size()){
                Reset();
            }
            break;
        default:
            _state = fill;
    }  
}

size_t FallingSand::_GetColorIndex(size_t offset) {
    return (offset / _bar_size) % _colors.size();
}

double FallingSand::_PositiveFmod(double numerator, double denominator) {
    return fmod(fmod(numerator, denominator) + denominator, denominator);
}

frc::Color FallingSand::_CorrectGamma(frc::Color color) {
    return frc::Color(std::pow(color.red, _gamma), std::pow(color.green, _gamma), std::pow(color.blue, _gamma));
}