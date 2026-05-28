#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <Arduino.h>

class BatteryMonitor {
public:
  BatteryMonitor(
    uint8_t adcPin,
    float r1,
    float r2,
    uint32_t sampleIntervalMs = 10,
    uint16_t samplesToAverage = 32,
    float calibrationFactor = 1.0f
  );

  void begin();
  int update();

  int getPercent() const;
  float getVoltage() const;
  float getRawVoltage() const;

private:
  uint8_t _adcPin;
  float _r1, _r2;
  uint32_t _sampleIntervalMs;
  uint16_t _samplesToAverage;
  float _calibrationFactor;

  uint32_t _lastSampleMs;
  uint32_t _sumMilliVolts;
  uint16_t _sampleCount;

  float _rawVoltage;
  float _filteredVoltage;
  bool _initializedFilter;

  int _lastPercent;

  int voltageToPercentStep(float v);
};

void initBattery();
int readBatteryCharge();
float readBatteryVoltage();
float readBatteryRawVoltage();

#endif