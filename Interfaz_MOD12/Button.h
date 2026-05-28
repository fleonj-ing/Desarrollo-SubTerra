#ifndef BUTTON_H
#define BUTTON_H

#include <Arduino.h>

class Button {
  private:
    uint8_t pin;

    bool currentState;
    bool lastReading;
    bool lastStableState;

    unsigned long lastDebounceTime;
    unsigned long pressStartTime;
    unsigned long lastRepeatTime;

    const unsigned long debounceDelay = 30;
    const unsigned long holdDelay = 1000;
    const unsigned long repeatInterval = 700;

  public:
    Button(uint8_t p);

    void begin();
    void update();

    bool wasPressed();   // click
    bool isPressed();    // mantenido
    bool isRepeating();  // auto repeat
};

#endif
