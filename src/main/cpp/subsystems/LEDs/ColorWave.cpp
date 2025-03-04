#include "subsystems/LEDs/ColorWave.h"

ColorWave::ColorWave(std::vector<frc::Color> colors, units::meter_t led_spacing, units::meter_t wavelength, double gamma, units::meters_per_second_t velocity) {
    _colors = colors;
    _wavelength = wavelength/led_spacing;
    _gamma = gamma;
    _velocity = velocity / led_spacing;
    _timer.Start();
}

void ColorWave::ApplyTo(std::span<frc::AddressableLED::LEDData> data) {
    for (size_t i = 0; i < data.size(); i++) {
        data[i].SetLED(_ApplyBrightness(_colors[_GetColorIndex(i)], _GetBrightness(i)));
    }
}

double ColorWave::_GetBrightness(int offset) {
    return (1 - std::cos((2 * M_PI / _wavelength) * (offset - _WavePosition()))) / 2;
}

frc::Color ColorWave::_ApplyBrightness(frc::Color color, double brightness) {
    return frc::Color(_GammaCorrection(color.red * brightness), _GammaCorrection(color.green * brightness), _GammaCorrection(color.blue * brightness));
}

double ColorWave::_GammaCorrection(double brightness) {
    return std::pow(brightness, _gamma);
}

int ColorWave::_GetColorIndex(int offset) {
    return int(_PositiveFmod(floor((double (offset) - _WavePosition()) / _wavelength), _colors.size()));
}

double ColorWave::_PositiveFmod(double numerator, double denominator) {
    return fmod(fmod(numerator, denominator) + denominator, denominator);
}

double ColorWave::_WavePosition() {
    return _timer.Get() * _velocity;
}