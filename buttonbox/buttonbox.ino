#include <Joystick.h>

const int buttonPins[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23};

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_GAMEPAD,
                  21, 0,                 // Button Count, Hat Switch Count
                  true, true, false,     // X and Y, but no Z Axis
                  false, false, false,   // No Rx, Ry, or Rz
                  false, false,          // No rudder or throttle
                  false, false, false);  // No accelerator, brake, or steering

void setup() {
    for (int i = 0; i < (sizeof(buttonPins) / sizeof(buttonPins[0])); i++) {
        pinMode(buttonPins[i], INPUT_PULLUP);
    }
    Joystick.begin();
}

void loop() {
    for (int i = 0; i < (sizeof(buttonPins) / sizeof(buttonPins[0])); i++) {
        Joystick.setButton(i, digitalRead(buttonPins[i]) == LOW);
    }
    Joystick.sendState();
    delay(50);
}

