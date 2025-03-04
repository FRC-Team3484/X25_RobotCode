#ifndef FIRE_H
#define FIRE_H

#include <frc/LEDPattern.h>
#include <frc/AddressableLED.h>

class Fire {
    public:
        Fire(int height, int sparks, int delay, int n_leds);

        void ApplyTo(std::span<frc::AddressableLED::LEDData> data);
        void Reset();

    private:
        int _height;
        int _sparks;
        int _delay;
        unsigned int _size;
        unsigned char* _heat;

        int _Random(int low, int high);
        frc::Color _HeatColor(int tempature);
};

#endif