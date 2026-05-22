#include "Button.h"

Button::Button(uint8_t p) {
  pin = p;
}

void Button::begin() {
  pinMode(pin, INPUT);

  currentState = LOW;
  lastReading = LOW;
  lastStableState = LOW;

  lastDebounceTime = 0;
  pressStartTime = 0;
  lastRepeatTime = 0;
}

void Button::update() {
  bool reading = digitalRead(pin);

  if (reading != lastReading) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    currentState = reading;
  }

  lastReading = reading;
}

bool Button::wasPressed() {

  if (currentState == HIGH && lastStableState == LOW) {
    lastStableState = HIGH;
    return true;
  }

  if (currentState == LOW) {
    lastStableState = LOW;
  }

  return false;
}

bool Button::isPressed() {
  return currentState == HIGH;
}

bool Button::isRepeating() {

  if (currentState == HIGH) {

    if (pressStartTime == 0) {
      pressStartTime = millis();
      lastRepeatTime = millis();
      return true;
    }

    if (millis() - pressStartTime >= holdDelay) {
      if (millis() - lastRepeatTime >= repeatInterval) {
        lastRepeatTime = millis();
        return true;
      }
    }

  } else {
    pressStartTime = 0;
  }

  return false;
}
