#include <Arduino.h>

/*
  Clase utilitaria para lectura robusta de botones físicos.
  Objetivo:
  - Eliminar rebote eléctrico (debounce).
  - Exponer eventos de alto nivel:
      wasPressed()  -> flanco de subida (click único)
      isPressed()   -> estado sostenido
      isRepeating() -> autorepetición tras mantener presionado
  Esta abstracción simplifica la lógica de menús y captura de parámetros.
*/
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
