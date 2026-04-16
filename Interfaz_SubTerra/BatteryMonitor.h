#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <Arduino.h>
/*
  BatteryMonitor encapsula la adquisición y estimación de estado de carga:
  - Muestrea ADC en intervalos configurables.
  - Promedia N muestras para reducir ruido.
  - Reconstruye voltaje real usando divisor resistivo.
  - Aplica filtro EMA para estabilizar lectura.
  - Convierte voltaje a porcentaje discreto con histéresis.
*/
class BatteryMonitor {
public:
  BatteryMonitor(
    uint8_t adcPin,
    float r1,
    float r2,
    uint32_t sampleIntervalMs = 10,
    uint16_t samplesToAverage = 32,
    float calibrationFactor = 1.0f,
    float adcReferenceVoltage = 3.3f,
    uint16_t adcMaxValue = 4095
  );

  void begin();
  int update();

  int getPercent() const;
  float getVoltage() const;
  float getRawVoltage() const;

private:
  // Configuración física/electrónica del canal de batería.
  uint8_t _adcPin;
  float _r1, _r2;
  uint32_t _sampleIntervalMs;
  uint16_t _samplesToAverage;  
  float _calibrationFactor;
  float _adcReferenceVoltage;
  uint16_t _adcMaxValue;

  // Acumuladores de muestreo.
  uint32_t _lastSampleMs;
  uint32_t _sumMilliVolts;
  uint16_t _sampleCount;

  // Señales filtradas y estado del estimador.
  float _rawVoltage;
  float _filteredVoltage;
  bool _initializedFilter;

  // Último porcentaje emitido (se usa en histéresis).
  int _lastPercent;

  int voltageToPercentStep(float v);
};

void initBattery();
int readBatteryCharge();
float readBatteryVoltage();
float readBatteryRawVoltage();

#endif