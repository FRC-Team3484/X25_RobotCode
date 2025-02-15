#ifndef COLOR_WAVE_H
#define COLOR_WAVE_H

#include <frc/LEDPattern.h> 
#include <frc/AddressableLED.h>
#include <vector>
#include <units/math.h>
#include <units/length.h>
#include <units/velocity.h>
#include <frc/Timer.h>


class ColorWave {
 public:
  ColorWave(std::vector<frc::Color> colors, units::meter_t led_spacing, units::meter_t wavelength, double gamma, units::meters_per_second_t velocity);

  void ApplyTo(std::span<frc::AddressableLED::LEDData> data);


  private:
    std::vector<frc::Color> _colors;

    double _wavelength;

    double _gamma;

    double _GammaCorrection(double brightness);

    double _GetBrightness(int offset);

    frc::Color _ApplyBrightness(frc::Color color, double brightness);

    int _GetColorIndex(int offset);

    units::hertz_t _velocity;

    double _PositiveFmod(double numerator, double denonomator);

    frc::Timer _timer;

    double _WavePosition();



};


#endif