#include "Button.h"

// Constructor: solo almacena pin; la configuración eléctrica se hace en begin().
Button::Button(uint8_t p) {
  pin = p;
}

void Button::begin() {
  // Configuración inicial y limpieza de estados internos del antirrebote.
  pinMode(pin, INPUT);

  currentState = LOW;
  lastReading = LOW;
  lastStableState = LOW;

  lastDebounceTime = 0;
  pressStartTime = 0;
  lastRepeatTime = 0;
}

void Button::update() {
  // Actualiza estado estable usando ventana temporal de debounce.
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
  // Detecta transición LOW->HIGH una sola vez por pulsación.

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
  // Estado sostenido actual, útil para acciones continuas.
  return currentState == HIGH;
}

bool Button::isRepeating() {
  // Genera "ticks" periódicos cuando el botón se mantiene presionado.

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
