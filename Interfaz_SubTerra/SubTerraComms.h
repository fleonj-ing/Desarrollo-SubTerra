#pragma once

#include <Arduino.h>

/*
================================================================================
SubTerraComms
--------------------------------------------------------------------------------
Esta librería mantiene el protocolo binario robusto completo (SOF, VER, TYPE,
LEN, SEQ, PAYLOAD y CRC16), pero expone hacia la aplicación una interfaz simple
basada en polling para recibir SOLO el payload ya interpretado.

Flujo recomendado:
1) El código principal llama pollPayload(...) en su loop.
2) La librería consume bytes UART y reconstruye frames robustos.
3) La librería valida CRC/VER/TYPE/LEN.
4) Si el frame es válido, traduce el payload a campos semánticos:
   - cmd
   - hasValue / value
   - hasTimestamp / timestampUs
5) El código principal decide qué hacer con esos campos.
================================================================================
*/
class SubTerraComms {
 public:
  // Límite de seguridad de payload (evita sobrelectura/buffer overflow).
  static const uint8_t MAX_PAYLOAD = 32;
  static const uint8_t TYPE_CMD = 0x10;

  // Catálogo de comandos del sistema SubTerra (interfaz y sonda).
  enum CommandCode : uint16_t {
    CMD_ENTER_MANUAL      = 100,
    CMD_ENTER_CONTROLADO  = 200,
    CMD_ENTER_AUTOMATICO  = 300,
    CMD_ENTER_PRUEBA      = 400,
    CMD_ENTER_MENU        = 500,
    CMD_SET_RESOLUTION    = 88,
    CMD_START_SCAN        = 23,
    CMD_SCAN_FINISHED     = 32,
    CMD_AGUA_DETECT       = 33,
    CMD_TIERRA_DETECT     = 34,
    CMD_VL53_READY        = 35,
    CMD_VL53_NOT_READY    = 36,
    CMD_CANCEL_SELECT     = 37,
    CMD_VL53_STATUS_QUERY = 53,
    CMD_VL53_READING      = 54,
    CMD_LINK_PING         = 90,
    CMD_LINK_PONG         = 91,
    CMD_TIME_START        = 14,
    CMD_TIME_FINISH       = 41
  };

  // Resultado del parser robusto interno.
  enum ParseResult : uint8_t {
    PARSE_OK,
    PARSE_CRC_FAIL,
    PARSE_BAD_VER,
    PARSE_BAD_TYPE,
    PARSE_LEN_TOO_BIG,
    PARSE_INCOMPLETE,
    PARSE_DESYNC
  };

  // Estructura simple para la lógica de aplicación.
  struct DecodedPayload {
    uint16_t cmd;
    bool hasValue;
    uint16_t value;
    bool hasTimestamp;
    uint32_t timestampUs;
    uint8_t seq;
  };

  // Estructura de frame completa (útil para debug y diagnóstico).
  struct DecodedFrame {
    uint8_t ver;
    uint8_t type;
    uint8_t len;
    uint8_t seq;

    uint8_t payload[MAX_PAYLOAD];

    bool hasCommand;
    uint16_t cmd;
    bool hasValue;
    uint16_t value;
    bool hasTimestamp;
    uint32_t timestampUs;

    uint16_t crcRx;
    uint16_t crcCalc;

    uint8_t raw[2 + 4 + MAX_PAYLOAD + 2];
    uint8_t rawLen;
  };

  explicit SubTerraComms(HardwareSerial& serialPort);

  void begin(uint32_t baud, int8_t rxPin = -1, int8_t txPin = -1);

  // API de envío.
  void sendCommand(uint16_t cmd);
  void sendCommandWithValue(uint16_t cmd, uint16_t value);
  void sendCommandWithValueAndTime(uint16_t cmd, uint16_t value, uint32_t timestampUs);
  void sendEnterManual();
  void sendEnterControlado();
  void sendEnterAutomatico();
  void sendEnterPrueba();
  void sendEnterMenu();
  void sendStartScan();
  void sendResolutionValue(uint8_t resolution_1_to_10);
  void sendScanFinish();
  void sendAguaDetect();
  void sendCancelSelect();
  void sendTierraDetect();
  void sendVl53Ready();
  void sendVl53NotReady();
  void sendVl53StatusQuery();
  void sendVl53Reading(uint16_t distanceMm, uint32_t timestampUs);
  void sendLinkPing();
  void sendLinkPong();
  void sendTimeStart(uint32_t timestampUs);
  void sendTimeFinish(uint32_t timestampUs);

  /*
    API principal de recepción (polling/manual).

    Qué devuelve:
    - true: se recibió un payload de comando válido y outPayload quedó lleno.
    - false: no hubo payload nuevo válido en esta llamada.

    Opcional:
    - outLastParse permite saber el último estado del parser robusto.
      (PARSE_INCOMPLETE cuando no hay frame completo todavía).
  */
  bool pollPayload(DecodedPayload& outPayload, ParseResult* outLastParse = nullptr);

  // Parser robusto de bajo nivel (disponible para diagnóstico).
  ParseResult readSerialDuplex(DecodedFrame& outFrame);

  static bool isVl53ReadingFrame(const DecodedFrame& frame, uint16_t& distanceMm, uint32_t& timestampUs);
  void formatDecodedFrame(const DecodedFrame& frame, char* outText, size_t outSize) const;

 private:
  enum RxState : uint8_t {
    WAIT_SOF1,
    WAIT_SOF2,
    READ_VER,
    READ_TYPE,
    READ_LEN,
    READ_SEQ,
    READ_PAYLOAD,
    READ_CRC_L,
    READ_CRC_H
  };

  static const uint8_t SOF1 = 0xAA;
  static const uint8_t SOF2 = 0x55;
  static const uint8_t PROTO_VER = 0x01;

  HardwareSerial& serial;
  uint8_t txSeq;

  RxState rxState;
  uint8_t rxVer;
  uint8_t rxType;
  uint8_t rxLen;
  uint8_t rxSeq;
  uint8_t rxPos;
  uint16_t rxCrc;
  uint8_t rxPayload[MAX_PAYLOAD];
  uint8_t crcBuf[4 + MAX_PAYLOAD];
  bool desyncFlag;

  static uint16_t crc16Ccitt(const uint8_t* data, size_t len);
  static void putU16(uint8_t* p, uint16_t v);
  static void putU32(uint8_t* p, uint32_t v);
  static uint16_t getU16(const uint8_t* p);
  static uint32_t getU32(const uint8_t* p);

  void sendCommandWithTime(uint16_t cmd, uint32_t timestampUs);
  void sendFrame(uint8_t type, const uint8_t* payload, uint8_t len);

  static void clearDecodedFrame(DecodedFrame& f);
  static void clearDecodedPayload(DecodedPayload& p);
  const char* commandName(uint16_t cmd) const;
};