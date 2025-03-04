#include "subsystems/LEDs/Fire.h"

Fire::Fire(int height, int sparks, int delay, int n_leds) {
    /*
    @param height: The height of the fire. Smaller values are taller flames.
    @param sparks: The number of sparks to ignite. Higher values are more frequent sparks.
    @param delay: The delay between sparks. Smaller values are shorter delays.
    */
    _height = height;
    _sparks = sparks;
    _delay = delay;
    _size = n_leds;
    _heat = new unsigned char[n_leds];
    for (int i = 0; i < n_leds; i++) {
        _heat[i] = 0;
    }
}

void Fire::ApplyTo(std::span<frc::AddressableLED::LEDData> data) {
    unsigned int cooldown;
    

    // Cool down every cell a little
    for (unsigned int i = 0; i < _size; i++) {
        cooldown = _Random(0, ((_height * 10) / _size) + 2);

        if (cooldown > _heat[i]) {
            _heat[i] = 0;
        } else {
            _heat[i] = _heat[i] - cooldown;
        }
    }

    // Heat from below drifts up and diffuses a little
    for (unsigned int k = _size - 1; k >= 2; k--) {
        _heat[k] = (_heat[k - 1] + _heat[k - 2] + _heat[k - 2]) / 3;
    }

    // Randomly ignite new 'sparks' of heat near the bottom
    if (_Random(0, 255) < _sparks) {
        int y = _Random(0, 7);
        _heat[y] = _Random(160, 255);
    }

    // Map from heat cells to LED colors
    for (size_t j = 0; j < _size; j++) {
        data[j].SetLED(_HeatColor(_heat[j]));
    }
}

void Fire::Reset() {
    for (size_t i = 0; i < _size; i++) {
        _heat[i] = 0;
    }
}

frc::Color Fire::_HeatColor(int temperature) {
    // Scale 'heat' down from 0-255 to 0-191
    int t192 = (temperature * 192) / 255;

    // calculate ramp up from
    int heatramp = t192 & 0x3F; // 0..63
    heatramp <<= 2;             // scale up to 0..252

    // figure out which third of the spectrum we're in:
    if (t192 > 0x80) { // hottest 1/3
        return frc::Color{255, 255, heatramp};
    } else if (t192 > 0x40) { // middle third
        return frc::Color{255, heatramp, 0};
    } else { // coolest third
        return frc::Color{heatramp, 0, 0};
    }
}

int Fire::_Random(int low, int high) {
    return (std::rand() % (high - low)) + low;
}
