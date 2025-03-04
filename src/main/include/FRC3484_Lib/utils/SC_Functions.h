#ifndef SC_FUNCTIONS
#define SC_FUNCTIONS

#include <frc/LEDPattern.h>

const frc::Color SC_GammaCorrection(frc::Color color, double gamma) {
    return frc::Color(std::pow(color.red, gamma), std::pow(color.green, gamma), std::pow(color.blue, gamma));
}

#endif