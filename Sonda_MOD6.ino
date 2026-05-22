//  Liberias y Pines
  #include "SubTerraComms.h"
  #include "Adafruit_VL53L1X.h"
  #include "BatteryMonitor.h"

  Adafruit_VL53L1X vl53;

  static const uint8_t VL53_ADDR = 0x29;

  // Ajustes recomendados
  static const uint16_t TIMING_BUDGET_MS = 100;
  static const uint16_t INTERMEASUREMENT_MS = 100;


  //Puertos COM
  #define COMMS_RX_PIN 16
  #define COMMS_TX_PIN 17
  // Sensores
  #define SAS   34
  #define STA   36
  #define CNY70 39

  SubTerraComms comms(Serial2);
//  Variables COMM
  static volatile bool scanTerminadoDesdeSonda = true;
  static volatile bool hayNuevaLecturaVl53 = false;
  static volatile uint16_t vl53DistMm = 0;
  static volatile uint32_t vl53TimeUs = 0;
  // Último payload semántico recibido. Este bloque existe para que la lógica de
  // menú consulte variables simples (cmd, value, timestamp) sin tocar detalles
  // del frame robusto (SOF/VER/LEN/CRC/etc).
  static volatile bool lastPayloadHasValue = false;
  static volatile bool lastPayloadHasTimestamp = false;
  static volatile uint16_t lastPayloadCmd = 0;
  static volatile uint16_t lastPayloadValue = 0;
  static volatile uint32_t lastPayloadTimestampUs = 0;

  // Lee exactamente UN payload nuevo válido desde UART (si existe), ya decodificado.
  bool pollIncomingPayload(SubTerraComms::DecodedPayload& payload);

  // Lee exactamente UN payload nuevo válido desde UART (si existe), ya decodificado.
  bool readIncomingPayload(SubTerraComms::DecodedPayload& payload);

  // Aplica al programa el efecto del payload ya recibido.
  void applyPayloadEffects(const SubTerraComms::DecodedPayload& payload);

  // Drena el UART en modo polling para que no se acumulen frames pendientes.
  void processIncomingSerial();
/*  Variables Motor
  Control de motor DC con TB6612FNG
  - Canal A
  - Funciones reutilizables para llamar en loop()
  - Control de velocidad y dirección
  - Rampa suave opcional no bloqueante

  Compatible con Arduino / ESP32 / Teensy
  Ajusta los pines según tu hardware
  */

  const int PIN_PWMA = 19;   // Debe ser PWM
  const int PIN_AIN1 = 18;
  const int PIN_AIN2 = 5;
  const int PIN_STBY = 2;

  // Variables de control
  int currentSpeed = 0;      // velocidad actual: -255 a 255
  int targetSpeed = 0;       // velocidad objetivo: -255 a 255
  int resolucion = 80;

  // Variables para rampa suave
  unsigned long lastRampUpdate = 0;
  const unsigned long rampInterval = 10;  // ms entre pasos de rampa
  const int rampStep = 5;                 // cuánto cambia por paso
//

volatile bool cny70Detectado = false;
volatile bool cancelSel  = false;
volatile bool hayTierra  = false;
volatile bool hayAgua    = false;

const int pinG = 25;
const int pinB = 26;
const int pinR = 27;

const int freq = 5000;
const int resolution = 8;

void setup() {
  // put your setup code here, to run once:
  initBattery();
  motorBegin();
  pinMode(SAS,INPUT);
  pinMode(STA,INPUT);
  pinMode(CNY70,INPUT);
  Serial.begin(115200);
  comms.begin(9600, COMMS_RX_PIN, COMMS_TX_PIN);
  attachInterrupt(digitalPinToInterrupt(CNY70),readCny70,FALLING);
  
  Wire.begin();
  Serial.println("Inicializando VL53L1X...");
  if (!initVL53()) {
    Serial.println("Fallo al iniciar VL53L1X");
  }

  Serial.print("Timing budget (ms): ");
  Serial.println(vl53.getTimingBudget());
  Serial.println("VL53L1X listo");
  
  //ledcSetup(0, freq, resolution); // canal R
  //ledcSetup(1, freq, resolution); // canal G
  //ledcSetup(2, freq, resolution); // canal B

  ledcAttach(pinR, freq, resolution);
  ledcAttach(pinG, freq, resolution);
  ledcAttach(pinB, freq, resolution);
  //*
  motorSetSpeed(resolucion);
  delay(2000);
  cny70Detectado = false;
  while(!cny70Detectado){
    processIncomingSerial();
  }//*/
  motorBrake();
}

int cmd=0;
volatile bool vl53Scan = false;
volatile bool vl53StatusRequest = false;

void loop() {
  // put your main code here, to run repeatedly:
  processIncomingSerial();
  baterryCharge();
  hayAgua = digitalRead(SAS);
  hayTierra = digitalRead(STA);
  if(hayAgua){
    comms.sendAguaDetect();
    //comms.sendScanFinish();
    //break;
  }else if(hayTierra){
    comms.sendTierraDetect();
    //comms.sendScanFinish();
    //break;
  }else if(vl53StatusRequest){
    Wire.begin();
    Serial.println("Inicializando VL53L1X...");
    if (!initVL53()) {
      Serial.println("Fallo al iniciar VL53L1X");
      comms.sendVl53NotReady();
    }else{
      Serial.println("VL53L1X listo");
      comms.sendVl53Ready();
    }
    vl53StatusRequest = false;
  }else if(vl53Scan){
    uint8_t rangeStatus;
    int16_t mm;
    uint32_t t_us;
    detachInterrupt(digitalPinToInterrupt(CNY70));
    cny70Detectado = false;
    // Guardar tiempo inicial
    bool act = true;

    unsigned long t0 = millis();
    motorSetSpeed(resolucion);
    long tInicio = micros();
    comms.sendTimeStart(tInicio);
    while(!cny70Detectado && !cancelSel && !hayAgua && !hayTierra){
      processIncomingSerial();
      //Serial.println("MOTOR");// Mientras no pasen 800 ms, mantener interrupción desactivada
      if (millis() - t0 > 2600 && act) {
        // Reactivar interrupción después de 800 ms
        attachInterrupt(digitalPinToInterrupt(CNY70), readCny70, FALLING);
        while (digitalRead(CNY70) == HIGH) {
          // esperar a que el sensor ya no vea la marca
          processIncomingSerial();
          if (readReliableDistance(mm, t_us, rangeStatus)) {
            Serial.print("Raw mm: ");
            Serial.print(mm);
            Serial.print(" | TiemStrap: ");
            Serial.print(t_us);
            Serial.print(" | Status: ");
            Serial.print(rangeStatus);
            comms.sendVl53Reading(mm, t_us);
          }
        }
        Serial.println("Interrupcion reactivada");
        act = false;
      }
      Serial.println(cny70Detectado);
      if (readReliableDistance(mm, t_us, rangeStatus)) {
        Serial.print("Raw mm: ");
        Serial.print(mm);
        Serial.print(" | TiemStrap: ");
        Serial.print(t_us);
        Serial.print(" | Status: ");
        Serial.print(rangeStatus);
        comms.sendVl53Reading(mm, t_us);
      }
    }
    long tFinal = micros();
    motorBrake();
    vl53Scan = false;
    cancelSel = false;
    //comms.sendTimeStart(tInicio);
    comms.sendTimeFinish(tFinal);
    comms.sendScanFinish();
  }
/*
  Serial.println("CMD");
  
  if(analogRead(SAS) >= 450){
    // Send Agua
    //Serial.println("AGUA");
    comms.sendAguaDetect();
  }else if(analogRead(STA) == HIGH){
    // Send Tierra
    //Serial.println("TIERRA");
    comms.sendTierraDetect();
  }
  if(cmd == 23){ // Escanear 
    uint8_t rangeStatus;
    int16_t mm;
    uint32_t t_us;
    while(posi<=pulsesPerRev){
      //Serial.println("MOTOR");
      motorSetSpeed(200);
      if (readReliableDistance(mm, t_us, rangeStatus)) {
        Serial.print("Raw mm: ");
        Serial.print(mm);
        Serial.print(" | TiemStrap: ");
        Serial.print(t_us);
        Serial.print(" | Status: ");
        Serial.print(rangeStatus);
        comms.sendVl53Reading(mm, t_us);
      }
    }
    comms.sendScanFinish();
    motorBrake();
  }else if(cmd == 53){
    // Revisar VL53
    if (! vl53.begin(0x29, &Wire)) {
      Serial.print(F("Error on init of VL sensor: "));
      Serial.println(vl53.vl_status);
      //Serial.println("VL53 NO");
      comms.sendVl53NotReady();
    }else{
      Serial.println("VL53");
      comms.sendVl53Ready();
    }
  }else if(cmd == 90){
    // Send Pong
    //Serial.println("PONG");
    comms.sendLinkPong();
  }
//*/
}

bool initVL53() {
  if (!vl53.begin(VL53_ADDR, &Wire, false)) {
    Serial.print("Error begin(), vl_status=");
    Serial.println((int)vl53.vl_status);
    return false;
  }

  Serial.print("Sensor ID: 0x");
  Serial.println(vl53.sensorID(), HEX);

  // 1 = short, 2 = long en la API heredada usada por esta librería
  vl53.VL53L1X_SetDistanceMode(2);

  if (!vl53.setTimingBudget(TIMING_BUDGET_MS)) {
    Serial.println("No se pudo configurar timing budget");
    return false;
  }

  // Debe ser >= timing budget
  vl53.VL53L1X_SetInterMeasurementInMs(INTERMEASUREMENT_MS);

  if (!vl53.startRanging()) {
    Serial.print("Error startRanging(), vl_status=");
    Serial.println((int)vl53.vl_status);
    return false;
  }

  return true;
}
bool readReliableDistance(int16_t &mm, uint32_t &t_us, uint8_t &rangeStatus) {
  if (!vl53.dataReady()) return false;

  mm = vl53.distance();
  t_us = micros();
  vl53.VL53L1X_GetRangeStatus(&rangeStatus);
  vl53.clearInterrupt();

  // Acepta solo status válido
  if (rangeStatus != 0) return false;
  // Filtro físico básico
  if (mm < 30 || mm > 4000) return false;

  return true;
}
bool readIncomingPayload(SubTerraComms::DecodedPayload& payload) {
  SubTerraComms::ParseResult parseStatus = SubTerraComms::PARSE_INCOMPLETE;
  return comms.pollPayload(payload, &parseStatus);
}
void applyPayloadEffects(const SubTerraComms::DecodedPayload& payload) {
  // Guardar siempre el último payload procesado
  lastPayloadCmd = payload.cmd;
  lastPayloadHasValue = payload.hasValue;
  lastPayloadValue = payload.value;
  lastPayloadHasTimestamp = payload.hasTimestamp;
  lastPayloadTimestampUs = payload.timestampUs;

  switch (payload.cmd) {
    case SubTerraComms::CMD_ENTER_MANUAL:
      // opcional: ModManual = true;
      Serial.println("[MENU] RX CMD 100 -> Modo manual");
      break;

    case SubTerraComms::CMD_ENTER_CONTROLADO:
      // opcional: ModControaldo = true;
      Serial.println("[MENU] RX CMD 200 -> Modo controlado");
      break;

    case SubTerraComms::CMD_ENTER_AUTOMATICO:
      // opcional: ModAuto = true;
      Serial.println("[MENU] RX CMD 300 -> Modo automatico");
      break;

    case SubTerraComms::CMD_ENTER_PRUEBA:
      // opcional: ModPrueba = true;
      Serial.println("[MENU] RX CMD 400 -> Modo prueba sistema");
      break;

    case SubTerraComms::CMD_START_SCAN:
      vl53Scan = true;
      Serial.println("[MENU] RX CMD 23 -> Comenzar escaneo");
      break;

    case SubTerraComms::CMD_VL53_STATUS_QUERY:
      vl53StatusRequest = true;
      Serial.print("[MENU] RX CMD 53 -> Estado de VL53");
      break;

    case SubTerraComms::CMD_LINK_PING:
      // opcional: linkPingRecibido = true;
      comms.sendLinkPong();
      Serial.println("[MENU] RX CMD 90 -> Link ping");
      break;

    case SubTerraComms::CMD_SET_RESOLUTION:
      // opcional: linkPingRecibido = true;
      Serial.println("[MENU] RX CMD 88 -> Set resolucion");
      if(payload.hasValue){
        //resolucion = payload.value;
        resolucion = 40 + (payload.value - 1) * (255 - 40) / 9;
        Serial.print("Resolucion: ");Serial.println(resolucion);
      }
      break;

    case SubTerraComms::CMD_ENTER_MENU:
      // opcional: linkPingRecibido = true;
      resolucion = 80;
      Serial.println("[MENU] RX CMD 500 -> Set resolucion");
      break;

    case SubTerraComms::CMD_CANCEL_SELECT:
      cancelSel = true;
      Serial.println("[MENU] RX CMD 33 -> Cancel Select");
      break;

    default:
      Serial.printf("[MENU] RX CMD %u\n", (unsigned)payload.cmd);
      break;
  }
}
void processIncomingSerial() {
  SubTerraComms::DecodedPayload payload;
  while (readIncomingPayload(payload)) {
    applyPayloadEffects(payload);
  }
}
void readCny70() {
  cny70Detectado = true;
}
void baterryCharge(){
  int batteryPct = readBatteryCharge();
  if(batteryPct == 100){
    setColor(0, 255, 0);   // verde
  }else if(batteryPct == 75){
    setColor(0, 255, 0);   // verde
  }else if(batteryPct == 50){
    setColor(255, 255, 0); // amarillo
  }else if(batteryPct == 25){
    setColor(255, 255, 0); // amarillo
  }else if(batteryPct == 0){
    setColor(255, 0, 0);   // rojo
  }else{
    setColor(255, 255, 255); // blanco
  }
}

void setColor(int r, int g, int b) {
  ledcWrite(pinR, r); // rojo
  ledcWrite(pinG, g); // verde
  ledcWrite(pinB, b); // azul
}

// --------------------------------------------------
// Inicialización
  // --------------------------------------------------
  void motorBegin() {
    pinMode(PIN_PWMA, OUTPUT);
    pinMode(PIN_AIN1, OUTPUT);
    pinMode(PIN_AIN2, OUTPUT);
    pinMode(PIN_STBY, OUTPUT);
    // En Teensy, subir frecuencia PWM evita pitido audible del motor/driver.
    //analogWriteResolution(8);
    //analogWriteFrequency(PIN_PWMA, 20000);

    digitalWrite(PIN_STBY, HIGH); // activar driver
    motorStop();                  // iniciar detenido
  }

  // --------------------------------------------------
  // Función interna: aplica una velocidad al driver
  // speed: -255 a 255
  //   positivo = adelante
  //   negativo = reversa
  //   0 = stop libre
  // --------------------------------------------------
  void applyMotorSpeed(int speed) {
    speed = constrain(speed, -255, 255);

    if (speed > 0) {
      digitalWrite(PIN_AIN1, HIGH);
      digitalWrite(PIN_AIN2, LOW);
      analogWrite(PIN_PWMA, speed);
    }
    else if (speed < 0) {
      digitalWrite(PIN_AIN1, LOW);
      digitalWrite(PIN_AIN2, HIGH);
      analogWrite(PIN_PWMA, -speed);
    }
    else {
      // paro libre (coast)
      digitalWrite(PIN_AIN1, LOW);
      digitalWrite(PIN_AIN2, LOW);
      analogWrite(PIN_PWMA, 0);
    }

    currentSpeed = speed;
  }

  // --------------------------------------------------
  // Funciones públicas
  // --------------------------------------------------

  // Poner una velocidad directa inmediatamente
  void motorSetSpeed(int speed) {
    targetSpeed = constrain(speed, -255, 255);
    applyMotorSpeed(targetSpeed);
  }

  // Cambiar solo el objetivo, para que updateMotorRamp()
  // lo alcance poco a poco
  void motorSetTargetSpeed(int speed) {
    targetSpeed = constrain(speed, -255, 255);
  }

  // Paro libre: el motor gira por inercia hasta detenerse
  void motorStop() {
    targetSpeed = 0;
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, LOW);
    analogWrite(PIN_PWMA, 0);
    currentSpeed = 0;
  }

  // Frenado activo: detiene más rápido
  void motorBrake() {
    targetSpeed = 0;
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, HIGH);
    analogWrite(PIN_PWMA, 0);
    currentSpeed = 0;
  }

  // Poner el driver en standby
  void motorStandby() {
    digitalWrite(PIN_STBY, LOW);
  }

  // Sacar el driver de standby
  void motorWake() {
    digitalWrite(PIN_STBY, HIGH);
  }

  // --------------------------------------------------
  // Actualización no bloqueante para rampa suave
  // Llamar continuamente en loop()
  // --------------------------------------------------
  void updateMotorRamp() {
    unsigned long now = millis();

    if (now - lastRampUpdate < rampInterval) {
      return;
    }
    lastRampUpdate = now;

    if (currentSpeed < targetSpeed) {
      currentSpeed += rampStep;
      if (currentSpeed > targetSpeed) currentSpeed = targetSpeed;
      applyMotorSpeed(currentSpeed);
    }
    else if (currentSpeed > targetSpeed) {
      currentSpeed -= rampStep;
      if (currentSpeed < targetSpeed) currentSpeed = targetSpeed;
      applyMotorSpeed(currentSpeed);
    }
  }
/*
  // --------------------------------------------------
  // Ejemplo
  // --------------------------------------------------
  void setup() {
    Serial.begin(115200);
    motorBegin();
  }

  void loop() {
    // Siempre llamar esto si quieres cambio suave
    updateMotorRamp();

    // Ejemplo de secuencia automática
    static unsigned long t0 = millis();
    unsigned long t = millis() - t0;

    if (t < 3000) {
      // Adelante velocidad media
      motorSetTargetSpeed(150);
    }
    else if (t < 6000) {
      // Adelante más rápido
      motorSetTargetSpeed(255);
    }
    else if (t < 9000) {
      // Stop suave
      motorSetTargetSpeed(0);
    }
    else if (t < 12000) {
      // Reversa
      motorSetTargetSpeed(-180);
    }
    else if (t < 15000) {
      // Freno activo
      motorBrake();
    }
    else {
      t0 = millis(); // reiniciar ciclo
    }
  }//*/
//
