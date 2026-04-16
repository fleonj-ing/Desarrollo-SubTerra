#include "BatteryMonitor.h"

// ==========================
// CONFIGURACIÓN GLOBAL
// ==========================
/*
  Estas constantes describen el hardware de medición de batería:
  - Pin ADC
  - Divisor resistivo
  - Factor de calibración
  - Parámetros de muestreo y escala ADC
*/
// Pin ADC (usa ADC1 en ESP32 clásico: 32, 33, 34, 35, 36, 39)
static const uint8_t BAT_ADC_PIN = 15;

// Divisor resistivo:
// BAT+ --- R1 --- ADC_PIN --- R2 --- GND           ESP     TEENSY
static const float BAT_R1 = 2200.0f;   // ohms      2.2K    10k
static const float BAT_R2 = 8200.0f;   // ohms      8.2K    3.3k

// Factor de calibración con multímetro
static const float BAT_CAL = 1.000f;

// Muestreo
static const uint32_t BAT_SAMPLE_INTERVAL_MS = 10;
static const uint16_t BAT_SAMPLES_TO_AVERAGE = 32;

// Objeto global privado al módulo
static BatteryMonitor battery(
  BAT_ADC_PIN,
  BAT_R1,
  BAT_R2,
  BAT_SAMPLE_INTERVAL_MS,
  BAT_SAMPLES_TO_AVERAGE,
  BAT_CAL
);

BatteryMonitor::BatteryMonitor(
  uint8_t adcPin,
  float r1,
  float r2,
  uint32_t sampleIntervalMs,
  uint16_t samplesToAverage,
  float calibrationFactor
)
: _adcPin(adcPin),
  _r1(r1),
  _r2(r2),
  _sampleIntervalMs(sampleIntervalMs),
  _samplesToAverage(samplesToAverage),
  _calibrationFactor(calibrationFactor),
  _lastSampleMs(0),
  _sumMilliVolts(0),
  _sampleCount(0),
  _rawVoltage(0.0f),
  _filteredVoltage(0.0f),
  _initializedFilter(false),
  _lastPercent(100)   // arranca alto; luego se corrige solo
{
  // El estado arranca "conservadoramente alto" y converge en los primeros ciclos.
}

void BatteryMonitor::begin() {
  // Inicializa periférico ADC según plataforma.
  analogReadResolution(12);
  analogSetPinAttenuation(_adcPin, ADC_11db);
  _lastSampleMs = millis();
}

int BatteryMonitor::update() {
  /*
    Bucle incremental no bloqueante:
    1) toma muestra cuando vence el intervalo;
    2) acumula hasta completar ventana;
    3) calcula voltaje promedio;
    4) filtra y convierte a porcentaje.
  */
  uint32_t now = millis();

  if (now - _lastSampleMs >= _sampleIntervalMs) {
    _lastSampleMs = now;

    uint32_t mv = analogReadMilliVolts(_adcPin);
    _sumMilliVolts += mv;
    _sampleCount++;

    if (_sampleCount >= _samplesToAverage) {
      float avgMilliVolts = (float)_sumMilliVolts / (float)_sampleCount;
      float vAdc = avgMilliVolts / 1000.0f;

      // Reconstrucción del voltaje real de batería
      float vBat = vAdc * ((_r1 + _r2) / _r2) * _calibrationFactor;
      _rawVoltage = vBat;

      // Filtro EMA
      if (!_initializedFilter) {
        _filteredVoltage = vBat;
        _initializedFilter = true;
      } else {
        const float alpha = 0.2f;
        _filteredVoltage = alpha * vBat + (1.0f - alpha) * _filteredVoltage;
      }

      _lastPercent = voltageToPercentStep(_filteredVoltage);

      _sumMilliVolts = 0;
      _sampleCount = 0;
    }
  }

  return _lastPercent;
}

int BatteryMonitor::getPercent() const {
  // Último porcentaje calculado (con histéresis aplicada).
  return _lastPercent;
}

float BatteryMonitor::getVoltage() const {
  return _filteredVoltage;
}

float BatteryMonitor::getRawVoltage() const {
  return _rawVoltage;
}

// ======================================================
// HISTÉRESIS
// ======================================================
// Reglas:
// - Para subir, pide un voltaje mayor
// - Para bajar, pide un voltaje menor
//
// Estados: 100, 75, 50, 25, 0
//
// Umbrales propuestos para Li-ion 1S:
// 100% <-> 75% : baja a 75 si < 4.08 ; sube a 100 si >= 4.15
//  75% <-> 50% : baja a 50 si < 3.90 ; sube a 75  si >= 3.98
//  50% <-> 25% : baja a 25 si < 3.70 ; sube a 50  si >= 3.84
//  25% <->  0% : baja a 0  si < 3.40 ; sube a 25  si >= 3.50
//
// Entre esos rangos, se conserva el estado anterior.

int BatteryMonitor::voltageToPercentStep(float v) {
  // Conversión por bandas con histéresis para evitar oscilaciones visuales.
  switch (_lastPercent) {
    case 100:
      if (v < 4.08f) return 75;
      return 100;

    case 75:
      if (v >= 4.15f) return 100;
      if (v < 3.90f)  return 50;
      return 75;

    case 50:
      if (v >= 3.98f) return 75;
      if (v < 3.70f)  return 25;
      return 50;

    case 25:
      if (v >= 3.84f) return 50;
      if (v < 3.40f)  return 0;
      return 25;

    case 0:
      if (v >= 3.50f) return 25;
      return 0;

    default:
      // Si por algo cae en estado inválido, lo reasigna por umbral simple
      if (v >= 4.15f) return 100;
      if (v >= 3.98f) return 75;
      if (v >= 3.84f) return 50;
      if (v >= 3.50f) return 25;
      return 0;
  }
}

// ==========================
// FUNCIONES PÚBLICAS
// ==========================

void initBattery() {
  // API simple para sketches: inicializa monitor global.
  battery.begin();
}

int readBatteryCharge() {
  // Devuelve porcentaje actualizado sin exponer detalles internos.
  return battery.update();
}

float readBatteryVoltage() {
  return battery.getVoltage();
}

float readBatteryRawVoltage() {
  return battery.getRawVoltage();
}