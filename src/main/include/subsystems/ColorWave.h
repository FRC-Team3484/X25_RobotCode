#ifndef COLOR_WAVE_H
#define COLOR_WAVE_H

#include <frc/LEDPattern.h> 
#include <frc/AddressableLED.h>
#include <vector>



class ColorWave {
 public:
  ColorWave(std::vector<frc::Color> colors);

  void ApplyTo(std::span<frc::AddressableLED::LEDData> data);


  private:
    std::vector<frc::Color> _colors;


};


#endif