#include "subsystems/LEDs/ColorStack.h"

using namespace units;

ColorStack::ColorStack(std::vector<frc::Color> colors, int bar_size, units::meter_t led_spacing, units::meters_per_second_t velocity, size_t fill_size, size_t empty_size, double gamma) {
    _colors = colors;
    _bar_size = bar_size;
    _velocity = velocity.value();
    _fill_size = fill_size;
    _empty_size = empty_size;
    _gamma = gamma;
    for (frc::Color color : _colors) {
        _colors.push_back(_CorrectGamma(color));
    }
    Reset();
}

void ColorStack::Reset() {
    _leds_placed = 0;
    _falling_led_position = 0;
    _state = fill;
}

void ColorStack::ApplyTo(std::span<frc::AddressableLED::LEDData> data) {
    _falling_led_position += _velocity;
    switch (_state) {
        case fill:
            if(_leds_placed>=data.size()){
                _state =empty;
                _leds_placed = data.size();
                _falling_led_position = data.size();
            }
            if(_falling_led_position >= data.size()-_leds_placed){
                _leds_placed+= _fill_size;
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
            break;
        case empty:
            if(_leds_placed<=0){
                Reset();
            }
            if(_falling_led_position >= data.size()){
                _leds_placed -= _empty_size;
                _falling_led_position = _leds_placed+_PositiveFmod(_falling_led_position, double(_empty_size));
            }
            for(size_t i =0; i <data.size(); i++){
                if(i<_leds_placed){
                    data[i].SetLED(_colors[_GetColorIndex(i)]);
                }else if(i<=size_t(_falling_led_position) && i>size_t(_falling_led_position)-_empty_size){
                    data[i].SetLED(_colors[_GetColorIndex(_leds_placed + i - size_t(_falling_led_position))]);
                }else{
                    data[i].SetLED(frc::Color::kBlack);
                }
            }
            break;
        default:
            _state = fill;
    }
}

size_t ColorStack::_GetColorIndex(size_t offset) {
    return (offset / _bar_size) % _colors.size();
}

double ColorStack::_PositiveFmod(double numerator, double denominator) {
    return fmod(fmod(numerator, denominator) + denominator, denominator);
}

frc::Color ColorStack::_CorrectGamma(frc::Color color) {
    return frc::Color(std::pow(color.red, _gamma), std::pow(color.green, _gamma), std::pow(color.blue, _gamma));
}
