
#include "subsystems/ColorWave.h"

ColorWave::ColorWave(std::vector<frc::Color> colors) {
    _colors = colors;
}

void ColorWave::ApplyTo(std::span<frc::AddressableLED::LEDData> data) {
    for (size_t i = 0; i < data.size(); i++) {
        data[i].SetLED(_colors[0]);
    }
}
