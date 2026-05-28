#include "SubTerraComms.h"

SubTerraComms::SubTerraComms(HardwareSerial& serialPort)
    : serial(serialPort),
      txSeq(0),
      rxState(WAIT_SOF1),
      rxVer(0),
      rxType(0),
      rxLen(0),
      rxSeq(0),
      rxPos(0),
      rxCrc(0),
      desyncFlag(false) {}

void SubTerraComms::begin(uint32_t baud, int8_t rxPin, int8_t txPin) {
#if defined(ESP32)
  if (rxPin >= 0 && txPin >= 0) {
    serial.begin(baud, SERIAL_8N1, rxPin, txPin);
    return;
  }
#elif defined(CORE_TEENSY)
  if (rxPin >= 0) Serial1.setRX(rxPin);
  if (txPin >= 0) Serial1.setTX(txPin);
#endif
  serial.begin(baud);
}

uint16_t SubTerraComms::crc16Ccitt(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= ((uint16_t)data[i] << 8);
    for (int b = 0; b < 8; b++) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else             crc = (crc << 1);
    }
  }
  return crc;
}

void SubTerraComms::putU16(uint8_t* p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
}

void SubTerraComms::putU32(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
  p[2] = (uint8_t)((v >> 16) & 0xFF);
  p[3] = (uint8_t)((v >> 24) & 0xFF);
}

uint16_t SubTerraComms::getU16(const uint8_t* p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

uint32_t SubTerraComms::getU32(const uint8_t* p) {
  return (uint32_t)p[0] |
         ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

void SubTerraComms::sendFrame(uint8_t type, const uint8_t* payload, uint8_t len) {
  if (len > MAX_PAYLOAD) return;

  uint8_t frame[2 + 4 + MAX_PAYLOAD + 2];
  size_t idx = 0;

  frame[idx++] = SOF1;
  frame[idx++] = SOF2;
  frame[idx++] = PROTO_VER;
  frame[idx++] = type;
  frame[idx++] = len;
  frame[idx++] = txSeq;

  for (uint8_t i = 0; i < len; i++) frame[idx++] = payload[i];

  const uint16_t crc = crc16Ccitt(&frame[2], 4 + len);
  frame[idx++] = (uint8_t)(crc & 0xFF);
  frame[idx++] = (uint8_t)((crc >> 8) & 0xFF);

  serial.write(frame, idx);
  txSeq++;
}

void SubTerraComms::sendCommand(uint16_t cmd) {
  uint8_t payload[2];
  putU16(payload, cmd);
  sendFrame(TYPE_CMD, payload, sizeof(payload));
}

void SubTerraComms::sendCommandWithValue(uint16_t cmd, uint16_t value) {
  uint8_t payload[4];
  putU16(&payload[0], cmd);
  putU16(&payload[2], value);
  sendFrame(TYPE_CMD, payload, sizeof(payload));
}

void SubTerraComms::sendCommandWithValueAndTime(uint16_t cmd, uint16_t value, uint32_t timestampUs) {
  uint8_t payload[8];
  putU16(&payload[0], cmd);
  putU16(&payload[2], value);
  putU32(&payload[4], timestampUs);
  sendFrame(TYPE_CMD, payload, sizeof(payload));
}

void SubTerraComms::sendCommandWithTime(uint16_t cmd, uint32_t timestampUs) {
  uint8_t payload[6];
  putU16(&payload[0], cmd);
  putU32(&payload[2], timestampUs);
  sendFrame(TYPE_CMD, payload, sizeof(payload));
}

void SubTerraComms::sendEnterManual() { sendCommand(CMD_ENTER_MANUAL); }
void SubTerraComms::sendEnterControlado() { sendCommand(CMD_ENTER_CONTROLADO); }
void SubTerraComms::sendEnterAutomatico() { sendCommand(CMD_ENTER_AUTOMATICO); }
void SubTerraComms::sendEnterPrueba() { sendCommand(CMD_ENTER_PRUEBA); }
void SubTerraComms::sendEnterMenu() { sendCommand(CMD_ENTER_MENU); }
void SubTerraComms::sendStartScan() { sendCommand(CMD_START_SCAN); }
void SubTerraComms::sendScanFinish() { sendCommand(CMD_SCAN_FINISHED); }
void SubTerraComms::sendAguaDetect() { sendCommand(CMD_AGUA_DETECT); }
void SubTerraComms::sendCancelSelect() { sendCommand(CMD_CANCEL_SELECT); }
void SubTerraComms::sendTierraDetect() { sendCommand(CMD_TIERRA_DETECT); }
void SubTerraComms::sendVl53Ready() { sendCommand(CMD_VL53_READY); }
void SubTerraComms::sendVl53NotReady() { sendCommand(CMD_VL53_NOT_READY); }
void SubTerraComms::sendVl53StatusQuery() { sendCommand(CMD_VL53_STATUS_QUERY); }
void SubTerraComms::sendVl53Reading(uint16_t distanceMm, uint32_t timestampUs) {
  sendCommandWithValueAndTime(CMD_VL53_READING, distanceMm, timestampUs);}
void SubTerraComms::sendLinkPing() { sendCommand(CMD_LINK_PING); }
void SubTerraComms::sendLinkPong() { sendCommand(CMD_LINK_PONG); }
void SubTerraComms::sendResolutionValue(uint8_t resolution_1_to_10) {
  if (resolution_1_to_10 < 1 || resolution_1_to_10 > 8) return;
  sendCommandWithValue(CMD_SET_RESOLUTION, resolution_1_to_10);
}
void SubTerraComms::sendTimeStart(uint32_t timestampUs){
  sendCommandWithTime(CMD_TIME_START, timestampUs);
}
void SubTerraComms::sendTimeFinish(uint32_t timestampUs){
  sendCommandWithTime(CMD_TIME_FINISH, timestampUs);
}

void SubTerraComms::clearDecodedFrame(DecodedFrame& f) {
  f.ver = 0;
  f.type = 0;
  f.len = 0;
  f.seq = 0;

  f.hasCommand = false;
  f.cmd = 0;
  f.hasValue = false;
  f.value = 0;
  f.hasTimestamp = false;
  f.timestampUs = 0;

  f.crcRx = 0;
  f.crcCalc = 0;

  f.rawLen = 0;
}

void SubTerraComms::clearDecodedPayload(DecodedPayload& p) {
  p.cmd = 0;
  p.hasValue = false;
  p.value = 0;
  p.hasTimestamp = false;
  p.timestampUs = 0;
  p.seq = 0;
}

bool SubTerraComms::isVl53ReadingFrame(const DecodedFrame& frame, uint16_t& distanceMm, uint32_t& timestampUs) {
  if (!frame.hasCommand || frame.cmd != CMD_VL53_READING || !frame.hasValue || !frame.hasTimestamp) return false;
  distanceMm = frame.value;
  timestampUs = frame.timestampUs;
  return true;
}

bool SubTerraComms::pollPayload(DecodedPayload& outPayload, ParseResult* outLastParse) {
  clearDecodedPayload(outPayload);

  while (true) {
    DecodedFrame frame;
    const ParseResult result = readSerialDuplex(frame);
    if (outLastParse != nullptr) *outLastParse = result;

    // No hay frame completo todavía.
    if (result == PARSE_INCOMPLETE) return false;

    // Se recibió un frame robusto válido, pero sin comando decodificable.
    if (result == PARSE_OK && !frame.hasCommand) continue;

    // Se recibió un payload de comando listo para la lógica principal.
    if (result == PARSE_OK) {
      outPayload.cmd = frame.cmd;
      outPayload.hasValue = frame.hasValue;
      outPayload.value = frame.value;
      outPayload.hasTimestamp = frame.hasTimestamp;
      outPayload.timestampUs = frame.timestampUs;
      outPayload.seq = frame.seq;
      return true;
    }

    // Para CRC_FAIL/BAD_VER/BAD_TYPE/LEN_TOO_BIG/DESYNC seguimos leyendo
    // hasta encontrar un frame válido o quedarnos sin bytes.
  }
}

SubTerraComms::ParseResult SubTerraComms::readSerialDuplex(DecodedFrame& outFrame) {
  clearDecodedFrame(outFrame);

  if (desyncFlag) {
    desyncFlag = false;
    return PARSE_DESYNC;
  }

  while (serial.available() > 0) {
    const uint8_t b = (uint8_t)serial.read();

    switch (rxState) {
      case WAIT_SOF1:
        if (b == SOF1) {
          rxState = WAIT_SOF2;
          outFrame.raw[0] = SOF1;
          outFrame.rawLen = 1;
        } else {
          return PARSE_DESYNC;
        }
        break;

      case WAIT_SOF2:
        if (b == SOF2) {
          rxState = READ_VER;
          outFrame.raw[outFrame.rawLen++] = SOF2;
        } else {
          rxState = WAIT_SOF1;
          desyncFlag = true;
          return PARSE_DESYNC;
        }
        break;

      case READ_VER:
        rxVer = b;
        outFrame.raw[outFrame.rawLen++] = b;
        rxState = READ_TYPE;
        break;

      case READ_TYPE:
        rxType = b;
        outFrame.raw[outFrame.rawLen++] = b;
        rxState = READ_LEN;
        break;

      case READ_LEN:
        rxLen = b;
        outFrame.raw[outFrame.rawLen++] = b;

        if (rxLen > MAX_PAYLOAD) {
          rxState = WAIT_SOF1;
          return PARSE_LEN_TOO_BIG;
        }

        rxState = READ_SEQ;
        break;

      case READ_SEQ:
        rxSeq = b;
        outFrame.raw[outFrame.rawLen++] = b;
        rxPos = 0;
        rxState = (rxLen == 0) ? READ_CRC_L : READ_PAYLOAD;
        break;

      case READ_PAYLOAD:
        rxPayload[rxPos++] = b;
        outFrame.raw[outFrame.rawLen++] = b;
        if (rxPos >= rxLen) rxState = READ_CRC_L;
        break;

      case READ_CRC_L:
        rxCrc = (uint16_t)b;
        outFrame.raw[outFrame.rawLen++] = b;
        rxState = READ_CRC_H;
        break;

      case READ_CRC_H: {
        rxCrc |= ((uint16_t)b << 8);
        outFrame.raw[outFrame.rawLen++] = b;

        crcBuf[0] = rxVer;
        crcBuf[1] = rxType;
        crcBuf[2] = rxLen;
        crcBuf[3] = rxSeq;
        for (uint8_t i = 0; i < rxLen; i++) crcBuf[4 + i] = rxPayload[i];

        const uint16_t calc = crc16Ccitt(crcBuf, 4 + rxLen);

        outFrame.ver = rxVer;
        outFrame.type = rxType;
        outFrame.len = rxLen;
        outFrame.seq = rxSeq;
        outFrame.crcRx = rxCrc;
        outFrame.crcCalc = calc;

        for (uint8_t i = 0; i < rxLen; i++) outFrame.payload[i] = rxPayload[i];

        // Aquí se traduce el payload binario a variables útiles para aplicación.
        if (rxLen == 2) {
          outFrame.hasCommand = true;
          outFrame.cmd = getU16(&rxPayload[0]);
        } else if (rxLen == 4) {
          outFrame.hasCommand = true;
          outFrame.cmd = getU16(&rxPayload[0]);
          outFrame.hasValue = true;
          outFrame.value = getU16(&rxPayload[2]);
        } else if (rxLen == 6) {
          outFrame.hasCommand = true;
          outFrame.cmd = getU16(&rxPayload[0]);
          outFrame.hasTimestamp = true;
          outFrame.timestampUs = getU32(&rxPayload[2]);
        } else if (rxLen == 8) {
          outFrame.hasCommand = true;
          outFrame.cmd = getU16(&rxPayload[0]);
          outFrame.hasValue = true;
          outFrame.value = getU16(&rxPayload[2]);
          outFrame.hasTimestamp = true;
          outFrame.timestampUs = getU32(&rxPayload[4]);
        }

        rxState = WAIT_SOF1;

        if (calc != rxCrc) return PARSE_CRC_FAIL;
        if (rxVer != PROTO_VER) return PARSE_BAD_VER;
        if (rxType != TYPE_CMD) return PARSE_BAD_TYPE;

        return PARSE_OK;
      }
    }
  }

  return PARSE_INCOMPLETE;
}

const char* SubTerraComms::commandName(uint16_t cmd) const {
  switch (cmd) {
    case CMD_ENTER_MANUAL: return "CMD_ENTER_MANUAL";
    case CMD_ENTER_CONTROLADO: return "CMD_ENTER_CONTROLADO";
    case CMD_ENTER_AUTOMATICO: return "CMD_ENTER_AUTOMATICO";
    case CMD_ENTER_PRUEBA: return "CMD_ENTER_PRUEBA";
    case CMD_ENTER_MENU: return "CMD_ENTER_MENU";
    case CMD_START_SCAN: return "CMD_START_SCAN";
    case CMD_SCAN_FINISHED: return "CMD_SCAN_FINISHED";
    case CMD_AGUA_DETECT: return "CMD_AGUA_DETECT";
    case CMD_TIERRA_DETECT: return "CMD_TIERRA_DETECT";
    case CMD_VL53_READY: return "CMD_VL53_READY";
    case CMD_VL53_NOT_READY: return "CMD_VL53_NOT_READY";
    case CMD_VL53_STATUS_QUERY: return "CMD_VL53_STATUS_QUERY";
    case CMD_VL53_READING: return "CMD_VL53_READING";
    case CMD_LINK_PING: return "CMD_LINK_PING";
    case CMD_LINK_PONG: return "CMD_LINK_PONG";
    case CMD_TIME_START: return "CMD_TIME_START";
    case CMD_TIME_FINISH: return "CMD_TIME_FINISH";
    default: return "CMD_UNKNOWN";
  }
}

void SubTerraComms::formatDecodedFrame(const DecodedFrame& frame, char* outText, size_t outSize) const {
  if (outText == nullptr || outSize == 0) return;

  size_t pos = 0;

  for (uint8_t i = 0; i < frame.rawLen; i++) {
    const int n = snprintf(outText + pos, (pos < outSize) ? (outSize - pos) : 0,
                           "%02X%s", frame.raw[i], (i + 1 < frame.rawLen) ? " " : "");
    if (n <= 0) break;
    pos += (size_t)n;
    if (pos >= outSize) {
      outText[outSize - 1] = '\0';
      return;
    }
  }

  const bool crcOk = (frame.crcCalc == frame.crcRx);

  if (frame.hasCommand) {
    if (frame.hasTimestamp) {
      if (frame.hasValue) {
        snprintf(outText + pos, (pos < outSize) ? (outSize - pos) : 0,
                 " | %s (%u) val=%u t_us=%lu | CRC %s",
                 commandName(frame.cmd),
                 (unsigned)frame.cmd,
                 (unsigned)frame.value,
                 (unsigned long)frame.timestampUs,
                 crcOk ? "OK" : "FAIL");
      } else {
        snprintf(outText + pos, (pos < outSize) ? (outSize - pos) : 0,
                 " | %s (%u) t_us=%lu | CRC %s",
                 commandName(frame.cmd),
                 (unsigned)frame.cmd,
                 (unsigned long)frame.timestampUs,
                 crcOk ? "OK" : "FAIL");
      }
    } else if (frame.hasValue) {
      snprintf(outText + pos, (pos < outSize) ? (outSize - pos) : 0,
               " | %s (%u) val=%u | CRC %s",
               commandName(frame.cmd),
               (unsigned)frame.cmd,
               (unsigned)frame.value,
               crcOk ? "OK" : "FAIL");
    } else {
      snprintf(outText + pos, (pos < outSize) ? (outSize - pos) : 0,
               " | %s (%u) | CRC %s",
               commandName(frame.cmd),
               (unsigned)frame.cmd,
               crcOk ? "OK" : "FAIL");
    }
  } else {
    snprintf(outText + pos, (pos < outSize) ? (outSize - pos) : 0,
             " | CMD_UNPARSED | CRC %s",
             crcOk ? "OK" : "FAIL");
  }
}