//Pines
  #include <Adafruit_ILI9341.h>
  #include <Adafruit_GFX.h>
  #include <SdFat.h>
  #include "icons_16x16.h"
  #include "Button.h"
  #include "SubTerraComms.h"
  #include <stdarg.h>
  #include <stdio.h>
  #include <Encoder.h>
  #include "BatteryMonitor.h"

  // Pantalla tft
  #define TFT_DC 9
  #define TFT_CS 10
  // Micro SD
  #define SD_CS   2
  #define SD_MOSI 26
  #define SD_MISO 39
  #define SD_SCK  27
  // SD interna (SDIO) Teensy 4.1
  SdFs sdInt;
  // SD externa (SPI)
  SdFs sdExt;
  //Botones
  #define selectButton  25
  #define upButton      28
  #define downButton    3
  #define undoButton    16
  //Puertos COM
  #define COMMS_RX_PIN 0
  #define COMMS_TX_PIN 1
  // Limit switch
  #define LimSWT  4

  Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

  Button btnSelect(selectButton);
  Button btnUp(upButton);
  Button btnDown(downButton);
  Button btnUndo(undoButton);
//
//Variables COMM & Pixels
  const int posiManual     = 70;
  const int posiControlado = 100;
  const int posiAutomatico = 130;
  const int posiPrueba     = 160;
  const int xMenu          = 20;
  const int posiTitulo     = 30;
  const int xTitulo        = 13;

  // Teensy 4.1: pines 0/1 corresponden a Serial1 (UART1)
  SubTerraComms comms(Serial1);
  static volatile bool scanTerminadoDesdeSonda = true;
  static volatile bool hayAgua = false;
  static volatile bool hayTierra = false;
  static volatile bool vl53ReadyFlag = false;
  static volatile bool vl53NotReadyFlag = false;
  static volatile bool linkPongRecibido = false;
  static volatile bool hayNuevaLecturaVl53 = false;
  static volatile bool hayTiempoInicio = false;
  static volatile bool hayTiempoTotal = false;
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
  bool readIncomingPayload(SubTerraComms::DecodedPayload& payload);

  // Aplica al programa el efecto del payload ya recibido.
  void applyPayloadEffects(const SubTerraComms::DecodedPayload& payload);

  // Drena el UART en modo polling para que no se acumulen frames pendientes.
  void processIncomingSerial();
    
  // Declaración de la función
  bool obtenerNombreArchivo(char* nombreFinal, size_t len);
  bool crearCrudos();
  bool respaldarSdInterna();
  
  // Buffer donde guardaremos el nombre del archivo elegido
  char nombreArchivo[32];
  bool nombreArchivoListo = false; // true cuando crearCrudos() dejó un nombre válido listo para escritura
//
/*Variables Motor
  Control de motor DC con TB6612FNG
  - Canal A
  - Funciones reutilizables para llamar en loop()
  - Control de velocidad y dirección
  - Rampa suave opcional no bloqueante

  Compatible con Arduino / ESP32 / Teensy
  Ajusta los pines según tu hardware
  */

  const int PIN_PWMA = 37;   // Debe ser PWM
  const int PIN_AIN1 = 40;
  const int PIN_AIN2 = 38;
  //const int PIN_STBY = 2; //PENDIENTE
  Encoder myEnc(23, 22);

  // Variable de encoder
  long oldPosition  = -999;
  int dist3Cm = 3675; // PULSOS DEL ENCODER

  // Variables de control
  int currentSpeed = 145;      // velocidad actual: -255 a 255
  int targetSpeed = 0;       // velocidad objetivo: -255 a 255

  // Variables para rampa suave
  unsigned long lastRampUpdate = 0;
  const unsigned long rampInterval = 10;  // ms entre pasos de rampa
  const int rampStep = 1;                 // cuánto cambia por paso
  long newPosition = 0;
// Variables varias

  bool cienPct = false;
  bool tresPct = false;
  bool dosPct  = false;
  bool unoPct  = false;
  bool zeroPct = false;

  int SD_MHZ = 20;

  int queDist = 2;
  int barridos = 1;
  int flagUndo = 0;
  int flagBoton = 0;
  int resolucion = 5;
  int totalVueltasf = 0;
  int inicioControlado = 0;
  float profundidad = 0;

  ///////////////////////
  const float originX = -16;   // offset en X (mm)
  const float originY = 0.0;    // offset en Y
  int Z=0;
  int X ;  
  int Y ;
  unsigned long EscInicio = 0;
  unsigned long EscTotal = 0;
  bool regresando = false;

  const char archivoCrudo[] = "TempoTimeF.txt"; // archivo temporal de datos crudos
  
  bool finalDatos = false;
// -----------------------------

void setup() {
  // Inicializa el UART del RS-485 y parser robusto
  initBattery();
  motorBegin();
  Serial.begin(115200);
  comms.begin(115200, COMMS_RX_PIN, COMMS_TX_PIN);
  Serial.println("HOLA");

  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(ILI9341_WHITE);
  tft.drawBitmap(109,66, epd_bitmap_Logo, 102,107, 0x7945);
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  SPI1.setMOSI(SD_MOSI);
  SPI1.setMISO(SD_MISO);
  SPI1.setSCK(SD_SCK);
  SPI1.begin();
  // Arranca con velocidad BAJA (muchos módulos fallan a 25MHz)
  SdSpiConfig cfg(SD_CS, DEDICATED_SPI, SD_SCK_MHZ(SD_MHZ), &SPI1);
  unsigned long startTime = millis();
  bool sdOk = false;
  uint32_t lastSdTry = 0;
  uint32_t lastVl53Query = 0;
  uint32_t lastPing = 0;

  const uint32_t SD_RETRY_MS = 300;
  const uint32_t VL53_QUERY_MS = 200;
  const uint32_t PING_MS = 250;
  // Validacion Sistema
  while (millis() - startTime < 3000) {
    const uint32_t now = millis();

    // 1) SD: reintento espaciado
    if (!sdOk && (now - lastSdTry >= SD_RETRY_MS)) {
      lastSdTry = now;
      if (sdExt.begin(cfg) && sdInt.begin(SdioConfig(FIFO_SDIO))) {
        sdOk = true;
        /*
        while (true) {
          sprintf(filename, "ESC%d.txt", contador);
          if (!sdExt.exists(filename)) {
            break; // este no existe, se usará
          }
          contador++;
        }//*/
      }
    }

    // 2) VL53: consulta espaciada
    if (!vl53ReadyFlag && (now - lastVl53Query >= VL53_QUERY_MS)) {
      lastVl53Query = now;
      comms.sendVl53StatusQuery(); // CMD 53
    }

    // 3) Ping de enlace (independiente de vl53Ok)
    if (!linkPongRecibido && (now - lastPing >= PING_MS)) {
      lastPing = now;
      comms.sendLinkPing(); // CMD 90
    }

    // 4) Leer payloads decodificados y aplicar decisiones de aplicación.
    processIncomingSerial();
    
    if (sdOk && vl53ReadyFlag && linkPongRecibido) {
      break;
    }

    delay(2); // cede CPU y evita bucle agresivo
  }
  tft.fillScreen(UI_NAVY);
  tft.setTextColor(ILI9341_WHITE);
  // Dibujo de bitmaps
    if(sdOk){
      tft.drawBitmap(13, 4, epd_bitmap_SD,     16, 16, ILI9341_WHITE);
    }else{
      tft.drawBitmap(13, 4, epd_bitmap_WARNING, 16, 16, ILI9341_WHITE);
    }
    if(vl53ReadyFlag){
      tft.drawBitmap(35, 4, epd_bitmap_Sensor, 16, 16, ILI9341_WHITE);
    }else{
      tft.drawBitmap(35, 4, epd_bitmap_WARNING, 16, 16, ILI9341_WHITE);
    }
    if(linkPongRecibido){
      tft.drawBitmap(57, 4, epd_bitmap_Enlace, 16, 16, ILI9341_WHITE);
    }else{
      tft.drawBitmap(57, 4, epd_bitmap_WARNING, 16, 16, ILI9341_WHITE);
    }
  //
  baterryCharge();
  //tft.drawBitmap(275, 4, epd_bitmap_bateriaFull32, 32, 16, ILI9341_WHITE);

  btnDown.begin();
  btnUp.begin();
  btnSelect.begin();
  btnUndo.begin();

  ///////////////////////////////////////////////////
  //*
  tft.drawRect(7, 24, 306, 209, ILI9341_DARKGREY);
  tft.setTextSize(2);
  tft.setCursor(13,30);
  //        "|||||||||||||||||"  17 chr
  tft.print("Selector de modos");
  tft.drawFastHLine(10, 50, 300, ILI9341_DARKGREY);
  tft.setTextSize(3);
  printPaddedTextF(tft, xMenu, posiManual,
                  UI_NAVY, ILI9341_WHITE,
                  1, 1, 0, 0,
                  "Manual"); // L, T, R, B
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(xMenu,posiControlado);
  tft.print("Controlado");
  tft.setCursor(xMenu,posiAutomatico);
  tft.print("Automatico");
  tft.setCursor(xMenu,posiPrueba);
  tft.print("Prueba sistema");//*/
  
  ///////////////////////////////////////////////////
  /*
  tft.drawLine(0,0,240,170, ILI9341_YELLOW)
  void drawRect(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h, uint16_t color);//*/
}

///////////////////////////////////////   Pixel extra
/*void printPaddedText(Adafruit_ILI9341 &tft,
                     int16_t x, int16_t y,
                     const char *txt,
                     uint16_t fg, uint16_t bg,
                     int8_t padL, int8_t padT, int8_t padR, int8_t padB) {
  int16_t x1, y1;
  uint16_t w, h;

  // Mide el texto tal como se va a dibujar (respeta setTextSize y font actual)
  tft.getTextBounds(txt, x, y, &x1, &y1, &w, &h);

  // Fondo con padding (margen)
  int16_t rx = x1 - padL;
  int16_t ry = y1 - padT;
  int16_t rw = w + padL + padR;
  int16_t rh = h + padT + padB;

  tft.fillRect(rx, ry, rw, rh, bg);

  // Texto encima (sin fondo para que no recorte raro)
  tft.setCursor(x, y);
  tft.setTextColor(fg);
  tft.print(txt);
}*/
void printPaddedTextF(Adafruit_ILI9341 &tft,
                      int16_t x, int16_t y,
                      uint16_t fg, uint16_t bg,
                      int8_t padL, int8_t padT, int8_t padR, int8_t padB,
                      const char *format, ...) {

  // 1) Construye el texto final usando formato tipo printf
  char out[40];  // ajusta si necesitas strings más largos (40 suele bastar)

  va_list args;
  va_start(args, format);
  vsnprintf(out, sizeof(out), format, args);
  va_end(args);

  // 2) Mide el texto para saber qué área hay que limpiar/pintar
  int16_t x1, y1;
  uint16_t w, h;
  tft.getTextBounds(out, x, y, &x1, &y1, &w, &h);

  // 3) Calcula el rectángulo del fondo con padding
  int16_t rx = x1 - padL;
  int16_t ry = y1 - padT;
  int16_t rw = w + padL + padR;
  int16_t rh = h + padT + padB;

  // 4) Pinta fondo
  tft.fillRect(rx, ry, rw, rh, bg);

  // 5) Pinta texto
  tft.setCursor(x, y);
  tft.setTextColor(fg);
  tft.print(out);
}

int menu = 1;

bool controladoActivo = false;     // Si está true, la FSM toma control del loop
void loop() {

  // Siempre refrescar estados de botones
  btnSelect.update();
  btnUp.update();
  btnDown.update();
  btnUndo.update();
  processIncomingSerial();
  /*
  if (hayNuevaLecturaVl53) {
    const uint16_t dist = vl53DistMm;
    const uint32_t tUs = vl53TimeUs;
    hayNuevaLecturaVl53 = false;

    Serial.printf("[MENU] VL53 desde pollIncomingPayload mm=%u t_us=%lu\n",
                  (unsigned)dist,
                  (unsigned long)tUs);
  }//*/

  // Si el modo Controlado está activo, la FSM manda y salimos del loop normal
  if (controladoActivo) {
    fsm_controlado();
    return;
  }

  // -------- Lógica normal del menú (no bloqueante) --------
  if (btnDown.wasPressed()) {
    menu++;
    actualizarMenu();
  }

  if (btnUp.wasPressed()) {
    menu--;
    actualizarMenu();
  }

  if (btnSelect.wasPressed()) {
    actualizarPantalla();
  }
}
///////////////////////////////////////   Menu
void actualizarMenu() {
  tft.setTextSize(3);
  switch (menu) {
    case 0:   // DE ARRIBA A ABAJO
      menu = 4;
      actualizarMenu();
      break;
    case 1:   // MANUAL
      //        "||||||||||||||||"
      printPaddedTextF(tft, xMenu, posiAutomatico,
                       ILI9341_WHITE,UI_NAVY,
                      1, 1, 0, 0,
                      "Automatico"); // L, T, R, B
      printPaddedTextF(tft, xMenu, posiControlado,
                       ILI9341_WHITE,UI_NAVY,
                      1, 1, 0, 0,
                      "Controlado"); // L, T, R, B
      printPaddedTextF(tft, xMenu, posiPrueba,
                       ILI9341_WHITE,UI_NAVY,
                      1, 1, 0, 0,
                      "Prueba sistema"); // L, T, R, B
      printPaddedTextF(tft, xMenu, posiManual,
                       UI_NAVY,ILI9341_WHITE,
                      1, 1, 0, 0,
                      "Manual"); // L, T, R, B
      break;
    case 2:   // CONTROLADO
      printPaddedTextF(tft, xMenu, posiPrueba,
                       ILI9341_WHITE,UI_NAVY,
                      1, 1, 0, 0,
                      "Prueba sistema"); // L, T, R, B
      printPaddedTextF(tft, xMenu, posiManual,
                       ILI9341_WHITE,UI_NAVY,
                      1, 1, 0, 0,
                      "Manual"); // L, T, R, B
      printPaddedTextF(tft, xMenu, posiAutomatico,
                       ILI9341_WHITE,UI_NAVY,
                      1, 1, 0, 0,
                      "Automatico"); // L, T, R, B
      printPaddedTextF(tft, xMenu, posiControlado,
                       UI_NAVY,ILI9341_WHITE,
                      1, 1, 0, 0,
                      "Controlado"); // L, T, R, B
      break;
    case 3:   // AUTOMATICO
      printPaddedTextF(tft, xMenu, posiManual,
                       ILI9341_WHITE,UI_NAVY,
                      1, 1, 0, 0,
                      "Manual"); // L, T, R, B
      printPaddedTextF(tft, xMenu, posiControlado,
                       ILI9341_WHITE,UI_NAVY,
                      1, 1, 0, 0,
                      "Controlado"); // L, T, R, B
      printPaddedTextF(tft, xMenu, posiPrueba,
                       ILI9341_WHITE,UI_NAVY,
                      1, 1, 0, 0,
                      "Prueba sistema"); // L, T, R, B
      printPaddedTextF(tft, xMenu, posiAutomatico,
                       UI_NAVY,ILI9341_WHITE,
                      1, 1, 0, 0,
                      "Automatico"); // L, T, R, B
      break;
    case 4:   // PRUEBA SISTEMA
      printPaddedTextF(tft, xMenu, posiControlado,
                       ILI9341_WHITE,UI_NAVY,
                      1, 1, 0, 0,
                      "Controlado"); // L, T, R, B
      printPaddedTextF(tft, xMenu, posiAutomatico,
                       ILI9341_WHITE,UI_NAVY,
                      1, 1, 0, 0,
                      "Automatico"); // L, T, R, B
      printPaddedTextF(tft, xMenu, posiManual,
                       ILI9341_WHITE,UI_NAVY,
                      1, 1, 0, 0,
                      "Manual"); // L, T, R, B
      printPaddedTextF(tft, xMenu, posiPrueba,
                       UI_NAVY,ILI9341_WHITE,
                      1, 1, 0, 0,
                      "Prueba sistema"); // L, T, R, B
      break;
    case 5:   // DE ABAJO A ARRIBA
      menu = 1;
      actualizarMenu();
      break;
  }
}

void actualizarPantalla(){
  // Reinicia flags de evento "latched" al entrar a un modo nuevo.
  // Si agua/tierra quedan en true de una corrida anterior, bloquean
  // el envío de CMD_START_SCAN en manual/automático.
  hayAgua = false;
  hayTierra = false;
  hayNuevaLecturaVl53 = false;
  scanTerminadoDesdeSonda = true;

  switch (menu){
    case 1:   // MANUAL
      comms.sendEnterManual();
      esc_manual();
      break;
    case 2:   // CONTROLADO
      comms.sendEnterControlado();
      iniciar_controlado();
      //esc_controlado();
      break;
    case 3:   // AUTOMATICO
      comms.sendEnterAutomatico();
      esc_automatico();
      break;
    case 4:   // PRUEBA SISTEMA
      comms.sendEnterPrueba();
      prueba_sistema();
      break;
  }
}
///////////////////////////////////////   MODOS
//  ||LISTO|| AGREGAR Z+=??? - DETERMINAR EL VALOR EN CM DE CADA PULSO DEL ENCODER    NO ES PROFUNDIDAD CREAR VARIABLE CON FORMULA NUEVA ENCODER
void esc_manual(){
  Z = 0;
  myEnc.write(0);
  cambioModoTitulo();
  tft.print("Escaneo Manual");
  tft.setTextColor(ILI9341_WHITE,UI_NAVY);
  tft.setTextSize(4);
  tft.setCursor(32,88);
  tft.print(" En espera ");
  // espera a que se suelte cualquier botón antes de empezar a evaluar acciones
  //bool seleWasPress = false; 
  
  nombreArchivoListo = crearCrudos();
  // Estado limpio por cada entrada al modo manual.
  //hayAgua = false;
  //hayTierra = false;
  //scanTerminadoDesdeSonda = true;

  flushBotones();
    bool flagUp = true;
    bool flagDown = true;
    int positionManual = 0;
  while(true){
    positionManual = myEnc.read();
    float metros = positionManual / 105000.0;
    Z = metros * 1000.0;
    // refrescar estados en bucles bloqueantes
    btnSelect.update();
    btnUp.update();
    btnDown.update();
    btnUndo.update();
    processIncomingSerial();

    /*
    Serial.print("LEIDO: ");Serial.println(leido);
    Serial.print("LEIDO BUCLE: ");Serial.println(leidoBucle);//*/

    if (btnUndo.wasPressed() && scanTerminadoDesdeSonda == true) break;

    if(btnUp.isPressed() && scanTerminadoDesdeSonda == true){
      tft.setTextSize(3);
      tft.setCursor(120,132);
      tft.print(metros,2);
      tft.print(" m");
      tft.setTextSize(4);
      tft.setCursor(40,88);
              //"||||||||||"
      tft.print(" Subiendo ");
      if(flagUp){
        motorSetSpeed(-145);
        flagUp = false;
      }
      motorSetTargetSpeed(-255);
      updateMotorRamp();
      //moverMotor3Cm();  //  SOLO ENCENDER Y APAGAR MOTOR EN ESTA CONCATENACION
    }else if(hayAgua){
      motorBrake();
      tft.setTextSize(3);
      tft.setCursor(120,132);
      tft.print(metros,2);
      tft.print(" m");
      tft.setTextSize(4);
      tft.setCursor(40,88);
              //"||||||||||"
      tft.print("   AGUA    ");
        flagUp = true;
    }else if(hayTierra){
      motorBrake();
      tft.setTextSize(3);
      tft.setCursor(120,132);
      tft.print(metros,2);
      tft.print(" m");
      tft.setTextSize(4);
      tft.setCursor(40,88);
              //"||||||||||"
      tft.print("  TIERRA   ");
        flagUp = true;
    }else if(btnDown.isPressed() && scanTerminadoDesdeSonda == true && (!hayAgua && !hayTierra)){
      tft.setTextSize(3);
      tft.setCursor(120,132);
      tft.print(metros,2);
      tft.print(" m");
      tft.setTextSize(4);
      tft.setCursor(33,88);
              //"||||||||||"
      tft.print("  Bajando   ");
      if(flagDown){
        motorSetSpeed(145);
        flagDown = false;
      }
      motorSetTargetSpeed(255);
      updateMotorRamp();
      //moverMotor3Cm();
    }else if(btnSelect.wasPressed() && scanTerminadoDesdeSonda == true && (!hayAgua && !hayTierra)){
      comms.sendLinkPing(); // comando 90
      linkPongRecibido = false;
      unsigned long inicioTimeout = millis();
      while (millis() - inicioTimeout < 1000) {
        processIncomingSerial();
      }
      if(!linkPongRecibido){
        tft.setTextSize(2);
        printPaddedTextF(tft, xMenu, 88,
                        ILI9341_WHITE,UI_NAVY, 
                        1, 1, 0, 0,
                        "Conexion fallida:  ");
        tft.fillRect(57, 4, 16, 16, UI_NAVY);
        tft.drawBitmap(57, 4, epd_bitmap_WARNING, 16, 16, ILI9341_WHITE);
        //regresando = true;
        //break;
      }else{
        comms.sendVl53StatusQuery(); // comando 53
        vl53NotReadyFlag = false;
        unsigned long inicioTimeout = millis();
        while (millis() - inicioTimeout < 1000) {
          processIncomingSerial();
        }
        if(vl53NotReadyFlag){
          tft.setTextSize(2);
          printPaddedTextF(tft, xMenu, 88,
                          ILI9341_WHITE,UI_NAVY, 
                          1, 1, 0, 0,
                          "Sensor fallando:   ");
          tft.fillRect(35, 4, 16, 16, UI_NAVY);
          tft.drawBitmap(35, 4, epd_bitmap_WARNING, 16, 16, ILI9341_WHITE);
          //regresando = true;
          //break;
        }else{
          scanTerminadoDesdeSonda = false;
          comms.sendStartScan(); // comando 23
          tft.setTextSize(3);
          tft.setCursor(120,132);
          tft.print(metros,2);
          tft.print(" m");
          tft.setTextSize(4);
          tft.setCursor(40,88);
                  //"||||||||||"
          tft.print("Escaneando");
        }
      }
    }else if(scanTerminadoDesdeSonda == false){
      // Solo intentamos guardar si la inicialización de archivo fue correcta.
      if (!nombreArchivoListo) {
        Serial.println("Advertencia: escaneo activo sin archivo de crudos inicializado.");
        delay(5);
        continue;
      }
      // Importante:
      // antes se abría el archivo en cada iteración y SOLO se cerraba cuando había lectura nueva,
      // lo que dejaba handles abiertos si no llegaba dato en ese ciclo.
      // Ahora abrimos/escribimos/cerramos únicamente cuando hay muestra disponible.
      if(hayNuevaLecturaVl53){
        FsFile crudo = sdInt.open(nombreArchivo, O_WRITE | O_APPEND | O_CREAT);
        if (!crudo) {
          Serial.print("Error al abrir archivo de crudos: ");
          Serial.println(nombreArchivo);
        } else {
          crudo.print(vl53TimeUs);
          crudo.print(" ");
          crudo.print(vl53DistMm);
          crudo.print(" ");
          crudo.println(Z);
          crudo.flush();
          crudo.close();
        }
        if(hayTiempoTotal){
          FsFile EscaneoTotales = sdInt.open(archivoCrudo, O_WRITE | O_APPEND | O_CREAT);
          EscaneoTotales.println(EscTotal);
          EscaneoTotales.flush();
          EscaneoTotales.close();
          hayTiempoTotal = false;
        }
        hayNuevaLecturaVl53 = false;
      }
      //saveSdExt();
      
    }else{
      tft.setTextSize(4);
      tft.setCursor(32,88);
      tft.print(" En espera ");
      baterryCharge();
        flagUp = true;
        flagDown = true;
      motorBrake();
      //if(!hayAgua && !hayTierra){
        //scanTerminadoDesdeSonda = true;
      //}
    }
  }
  
  myEnc.write(0);
  while(!finalDatos){
    saveSdExt();
  }
  finalDatos = false;
  
  menu = 1;
  comms.sendEnterMenu();
  cambioModoTitulo();
  tft.print("Selector de modos");
  actualizarMenu();
}

// ============================================================================
// 5) FSM (MAQUINA DE ESTADOS) PARA ESCANEO CONTROLADO
// ============================================================================

// 5.1) Lista de estados posibles del modo controlado
// Cada estado representa una "pantalla/paso" del asistente:
enum EstadoControlado {
  CTRL_DIST_INICIO,   // Paso 1: capturar distancia inicial
  CTRL_DIST_FINAL,    // Paso 2: capturar distancia final
  CTRL_RESOLUCION,    // Paso 3: seleccionar resolución
  CTRL_BARRIDOS,      // Paso 4: seleccionar cantidad de barridos
  CTRL_FIN            // Paso final: salir (o iniciar escaneo real)
};

// 5.2) Variables de control de la FSM
//ARRIBA------bool controladoActivo = false;     // Si está true, la FSM toma control del loop
EstadoControlado estadoCtrl;       // Estado actual (en qué paso vamos)
bool ctrlEntry = true;             // Se usa para ejecutar "cosas de entrada" UNA vez

// 5.3) Variables para guardar lo capturado (mejor que reusar una sola)
float distInicio = 0.00;
float distFinal  = 0.00;

// 5.4) Función que INICIA el modo (se llama una sola vez al entrar)
void iniciar_controlado() {

  // Activamos el modo controlado: desde ahora el loop correrá fsm_controlado()
  controladoActivo = true;

  // Estado inicial
  estadoCtrl = CTRL_DIST_INICIO;

  // Marcamos que acabamos de entrar a un estado (para dibujar una sola vez)
  ctrlEntry = true;

  // Reseteo de variables de captura
  profundidad = 0.00;
  distInicio = 0.00;
  distFinal = 0.00;

  resolucion = 4;
  barridos = 1;

  // Dibujo de layout general (igual a tu UI)
  cambioModoTitulo();
  tft.print("Escaneo Controlado");

  // “Marco” y separadores como tu código
  tft.drawFastVLine(160, 50, 183, ILI9341_DARKGREY);
  tft.drawFastHLine(7, 141, 306, ILI9341_DARKGREY);

  // Etiquetas
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);

  tft.setCursor(20, 60);   tft.print("Iniciar en:");
  tft.setCursor(174, 60);  tft.print("Resolucion:");
  tft.setCursor(28, 151);  tft.print("Finalizar:");
  tft.setCursor(185, 151); tft.print("Barridos:");

  // Valores iniciales visibles
  tft.setTextSize(3);
  tft.setCursor(20, 80);   tft.print("0.00 m");
  tft.setCursor(28, 171);  tft.print("0.00 m");
  tft.setCursor(174, 80);  tft.print("5");
  tft.setCursor(185, 171); tft.print("1");

  // IMPORTANTÍSIMO:
  // Si entraste a este modo presionando SELECT desde el menú,
  // aún puede estar físicamente presionado -> flasheos.
  flushBotones();
}

// 5.5) Función que EJECUTA la FSM (se llama en cada loop mientras esté activa)
void fsm_controlado() {

  // 1) Siempre actualiza botones al principio (FSM no funciona sin esto)
  btnSelect.update();
  btnUp.update();
  btnDown.update();
  btnUndo.update();

  // 2) La FSM decide qué hacer según el "estadoCtrl"
  switch (estadoCtrl) {

    // ==========================================================
    // ESTADO 1: Captura de Distancia Inicial
    // ==========================================================
    case CTRL_DIST_INICIO: {

      // --- Acción de entrada (solo 1 vez al entrar a este estado) ---
      if (ctrlEntry) {
        ctrlEntry = false;  // ya corrimos “entrada”
        profundidad = distInicio;

        // Visualmente: resaltar el campo de inicio
        // (fondo blanco, texto navy) como hacías tú
        printPaddedTextF(tft, 20, 80,
                        UI_NAVY, ILI9341_WHITE,
                        1,1,0,0,
                        "%.2f m",distInicio);
        printPaddedTextF(tft, 28, 171,
                        ILI9341_WHITE,UI_NAVY,
                        1,1,0,0,
                        "%.2f m",distFinal);
                        queDist = 1;
      }

      // --- Transición: si presiona Undo, salir completamente del modo ---
      if (btnUndo.wasPressed()) {
        // “Apagamos” el modo controlado y regresamos al menú
        controladoActivo = false;
        menu = 2;
        comms.sendEnterMenu();
        cambioModoTitulo();
        tft.print("Selector de modos");
        actualizarMenu();
        // aquí llamarías actualizarMenu() real en tu proyecto
        flushBotones();
        return; // salimos de la FSM este frame
      }

      // --- Lógica del estado (se ejecuta muchas veces, sin bloquear) ---
      // En este estado, UP/DOWN ajustan profundidad con tu regla tap/hold
      valorProf();

      // Si hubo cambio, actualizamos pantalla (solo cuando flagBoton=1)
      if (flagBoton) {
        printPaddedTextF(tft, 20, 80,
                        UI_NAVY, ILI9341_WHITE,
                        1,1,0,0,
                        "%.2f m",distInicio);
        flagBoton = 0;
      }

      // --- Transición: si presiona Select y profundidad válida, avanzar ---
      if (btnSelect.wasPressed() && profundidad >= 0.11) {

        // Ahora nos movemos al siguiente paso:
        estadoCtrl = CTRL_DIST_FINAL;

        // Marcamos que al entrar al siguiente estado ejecute su “entrada”
        ctrlEntry = true;

        flushBotones(); // evita heredar el select presionado
      }

      break;
    }

    // ==========================================================
    // ESTADO 2: Captura de Distancia Final
    // ==========================================================
    case CTRL_DIST_FINAL: {

      if (ctrlEntry) {
        ctrlEntry = false;
        if(distFinal<distInicio){
          profundidad = distInicio;
        }else{
          profundidad = distFinal;
        }

        // Resaltamos campo final (por ejemplo en la parte inferior)
        // y dejamos el inicio como “normal” (sin resaltado)
        // (Tú puedes ajustar exactamente qué campo resalta, aquí es el ejemplo)
        tft.setTextSize(3);

        // Mostrar inicio fijo arriba
        printPaddedTextF(tft, 20, 80,
                        ILI9341_WHITE,UI_NAVY,
                        1,1,0,0,
                        "%.2f m",distInicio);
        printPaddedTextF(tft, 174, 80,
                        ILI9341_WHITE,UI_NAVY, 
                        1,1,0,0,
                        "%d",resolucion);
        // Preparar captura de final desde 0 (recomendado)
        //profundidad = 0;
        // Resaltar final
        printPaddedTextF(tft, 28, 171,
                        UI_NAVY, ILI9341_WHITE,
                        1,1,0,0,
                        "%.2f m",profundidad);
                        queDist = 2;
      }

      // Undo aquí NO sale del modo; regresa al estado anterior (inicio)
      if (btnUndo.wasPressed()) {
        estadoCtrl = CTRL_DIST_INICIO;
        ctrlEntry = true;
        flushBotones();
        return;
      }

      valorProf();

      if (flagBoton) {
        printPaddedTextF(tft, 28, 171,
                        UI_NAVY, ILI9341_WHITE,
                        1,1,0,0,
                        "%.2f m",distFinal);
        flagBoton = 0;
      }

      if (btnSelect.wasPressed() && profundidad >= 0.11) {
        estadoCtrl = CTRL_RESOLUCION;
        ctrlEntry = true;
        flushBotones();
      }

      break;
    }

    // ==========================================================
    // ESTADO 3: Resolución
    // ==========================================================
    case CTRL_RESOLUCION: {

      if (ctrlEntry) {
        ctrlEntry = false;

        
        printPaddedTextF(tft, 28, 171,
                        ILI9341_WHITE,UI_NAVY, 
                        1,1,0,0,
                        "%.2f m",distFinal);
        // Resaltar barridos (abajo derecha)
        printPaddedTextF(tft, 185, 171,
                        ILI9341_WHITE,UI_NAVY, 
                        1,1,0,0,
                        "%d",barridos);
        // Resaltar el número de resolución (arriba derecha)
        printPaddedTextF(tft, 174, 80,
                        UI_NAVY, ILI9341_WHITE,
                        1,1,0,0,
                        "%d",resolucion);
      }

      // Undo: regresar a final
      if (btnUndo.wasPressed()) {
        estadoCtrl = CTRL_DIST_FINAL;
        ctrlEntry = true;
        flushBotones();
        return;
      }

      // Ajuste con repetición automática
      if (btnUp.isRepeating() && resolucion < 8) {
        resolucion++;
        flagBoton = 1;
      }
      if (btnDown.isRepeating() && resolucion > 1) {
        resolucion--;
        flagBoton = 1;
      }

      if (flagBoton) {
        printPaddedTextF(tft, 174, 80,
                        UI_NAVY, ILI9341_WHITE,
                        1,1,0,0,
                        "%d",resolucion);
        flagBoton = 0;
      }

      // Select: avanzar a barridos
      if (btnSelect.wasPressed()) {
        comms.sendResolutionValue((uint8_t)resolucion); // envía 1..8 según indicación
        estadoCtrl = CTRL_BARRIDOS;
        ctrlEntry = true;
        flushBotones();
      }
//
//
      break;
    }

    // ==========================================================
    // ESTADO 4: Barridos
    // ==========================================================
    case CTRL_BARRIDOS: {

      if (ctrlEntry) {
        ctrlEntry = false;

        printPaddedTextF(tft, 174, 80,
                        ILI9341_WHITE,UI_NAVY, 
                        1,1,0,0,
                        "%d",resolucion);

        // Resaltar barridos (abajo derecha)
        printPaddedTextF(tft, 185, 171,
                        UI_NAVY, ILI9341_WHITE,
                        1,1,0,0,
                        "%d",barridos);
      }

      // Undo: regresar a resolución
      if (btnUndo.wasPressed()) {
        estadoCtrl = CTRL_RESOLUCION;
        ctrlEntry = true;
        flushBotones();
        return;
      }

      // Ajuste con repetición
      if (btnUp.isRepeating()) {
        barridos++;
        flagBoton = 1;
      }
      if (btnDown.isRepeating() && barridos > 1) {
        barridos--;
        flagBoton = 1;
      }

      if (flagBoton) {
        printPaddedTextF(tft, 185, 171,
                        UI_NAVY, ILI9341_WHITE,
                        1,1,0,0,
                        "%d",barridos," ");
        flagBoton = 0;
      }

      // Select: terminar configuración
      if (btnSelect.wasPressed()) {
        estadoCtrl = CTRL_FIN;
        ctrlEntry = true;
        flushBotones();
      }

      break;
    }

    // ==========================================================
    // ESTADO 5: FIN
    // Aquí decides: ¿arrancar escaneo real? ¿mostrar resumen?
    // En este ejemplo: regresa al menú.
    // ==========================================================
    case CTRL_FIN: {

      if (ctrlEntry) {
        ctrlEntry = false;
        //scanTerminadoDesdeSonda = false;
        //comms.sendStartScan(); // comando 23
      }
      flagUndo = 0;
      regresando = false;
      nombreArchivoListo = crearCrudos();
      Z = inicioControlado * 35;

      // Aquí podrías hacer:
      // - iniciar motores
      // - guardar en SD
      // - pasar a un estado “ESCANEANDO”
      // etc.

      // Por ahora: salir al menú
      if (flagUndo == 0) {
        tft.fillRect(9, 52, 300, 177, UI_NAVY);
        tft.setTextColor(ILI9341_WHITE);
        tft.setTextSize(2);
        tft.setCursor(xMenu, 88);
        tft.print("Profundidad actual:");
        tft.setTextSize(3);
        tft.setCursor(xMenu, 120);
        tft.print(distInicio, 2);
        tft.print(" / ");
        tft.print(distFinal, 2);
        tft.print(" m");
        // 1) Siempre arrancar desde referencia superior.
        motorSetTargetSpeed(-255);
        while (digitalRead(LimSWT) == LOW) {
          updateMotorRamp();
          btnUndo.update();
          processIncomingSerial();
          if (btnUndo.wasPressed()) {
            flagUndo = 1;
            break;
          }
          updateMotorRamp();
        }
        motorStop();
      }
      // 2) Bajar hasta la profundidad de inicio definida en CTRL_DIST_INICIO.
      if (flagUndo == 0 && inicioControlado > 0) {
        long pulsosInicio = (long)inicioControlado * (long)dist3Cm;
        newPosition = 0;
        myEnc.write(0);
        motorSetTargetSpeed(255);
        while (newPosition < pulsosInicio && flagUndo == 0 && !hayAgua && !hayTierra) {
          btnSelect.update();
          btnUndo.update();
          processIncomingSerial();
          if (btnUndo.wasPressed() || btnSelect.wasPressed()) {
            flagUndo = 1;
            comms.sendCancelSelect();
            break;
          }
          updateMotorRamp();
          newPosition = myEnc.read();
        }
        motorBrake();
        myEnc.write(0);
      }

      // 3) Al detectar por encoder que llegó al inicio, ejecutar escaneo controlado.
      if (flagUndo == 0 && !hayAgua && !hayTierra) {
        int vueltas = inicioControlado;
        int barridoActual = 1;  // siempre inicia en 1

        scanTerminadoDesdeSonda = false;
        comms.sendStartScan(); // primer disparo en la profundidad inicial

        while ((barridoActual <= barridos) || !scanTerminadoDesdeSonda) {
          btnSelect.update();
          btnUndo.update();
          baterryCharge();
          processIncomingSerial();

          if (hayAgua) {
            tft.setTextSize(2);
            printPaddedTextF(tft, xMenu, 88,
                            ILI9341_WHITE,UI_NAVY,
                            1, 1, 0, 0,
                            "Agua detectada:    ");
            regresando = true;
            break;
          } else if (hayTierra) {
            tft.setTextSize(2);
            printPaddedTextF(tft, xMenu, 88,
                            ILI9341_WHITE,UI_NAVY,
                            1, 1, 0, 0,
                            "Tierra detectada:  ");
            regresando = true;
            break;
          } else if (btnSelect.wasPressed() || btnUndo.wasPressed()) {
            tft.setTextSize(2);
            printPaddedTextF(tft, xMenu, 88,
                            ILI9341_WHITE,UI_NAVY,
                            1, 1, 0, 0,
                            "Cancelacion manual:");
            comms.sendCancelSelect();
            regresando = true;
            break;
          } else if (scanTerminadoDesdeSonda == true) {
            bool barridoImpar = (barridoActual % 2) == 1;
            bool puedeMover = barridoImpar ? (vueltas < totalVueltasf) : (vueltas > inicioControlado);

            if (puedeMover) {
              // barrido impar: baja, barrido par: sube.
              if (barridoImpar) {
                moverMotor3Cm();
                vueltas++;
              } else {
                moverMotor3CmArriba();
                vueltas--;
              }

              Z = vueltas * 35;
              float alturaActual = 0.035 * (vueltas + 1) + 0.03;
              tft.fillRect(xMenu, 120, 290, 25, UI_NAVY);
              tft.setTextColor(ILI9341_WHITE);
              tft.setTextSize(3);
              tft.setCursor(xMenu, 120);
              tft.print(" ");
              tft.print(alturaActual);
              tft.print(" / ");
              tft.print(distFinal);
              tft.print(" m");

              comms.sendLinkPing(); // comando 90
              linkPongRecibido = false;
              unsigned long inicioTimeout = millis();
              while (millis() - inicioTimeout < 1000) {
                processIncomingSerial();
              }
              if(!linkPongRecibido){
                tft.setTextSize(2);
                printPaddedTextF(tft, xMenu, 88,
                                ILI9341_WHITE,UI_NAVY, 
                                1, 1, 0, 0,
                                "Conexion fallida:  ");
                tft.fillRect(57, 4, 16, 16, UI_NAVY);
                tft.drawBitmap(57, 4, epd_bitmap_WARNING, 16, 16, ILI9341_WHITE);
                regresando = true;
                break;
              }
              comms.sendVl53StatusQuery(); // comando 53
              vl53NotReadyFlag = false;
              inicioTimeout = millis();
              while (millis() - inicioTimeout < 1000) {
                processIncomingSerial();
              }
              if(vl53NotReadyFlag){
                tft.setTextSize(2);
                printPaddedTextF(tft, xMenu, 88,
                                ILI9341_WHITE,UI_NAVY, 
                                1, 1, 0, 0,
                                "Sensor fallando:   ");    
                tft.fillRect(35, 4, 16, 16, UI_NAVY);
                tft.drawBitmap(35, 4, epd_bitmap_WARNING, 16, 16, ILI9341_WHITE);
                regresando = true;
                break;
              }

              scanTerminadoDesdeSonda = false;
              comms.sendStartScan(); // comando 23
            } else {
              barridoActual++;
            }
          } else if (scanTerminadoDesdeSonda == false) {
            if (!nombreArchivoListo) {
              Serial.println("Advertencia: escaneo activo sin archivo de crudos inicializado.");
              delay(5);
              continue;
            }
            if (hayNuevaLecturaVl53) {
              FsFile crudo = sdInt.open(nombreArchivo, O_WRITE | O_APPEND | O_CREAT);
              if (!crudo) {
                Serial.print("Error al abrir archivo de crudos: ");
                Serial.println(nombreArchivo);
              } else {
                crudo.print(vl53TimeUs);
                crudo.print(" ");
                crudo.print(vl53DistMm);
                crudo.print(" ");
                crudo.println(Z);
                crudo.flush();
                crudo.close();
              }
              if (hayTiempoTotal) {
                FsFile EscaneoTotales = sdInt.open(archivoCrudo, O_WRITE | O_APPEND | O_CREAT);
                EscaneoTotales.println(EscTotal);
                EscaneoTotales.flush();
                EscaneoTotales.close();
                hayTiempoTotal = false;
              }
              hayNuevaLecturaVl53 = false;
            }
          }
        }
      }
      regresando = true;
      motorSetTargetSpeed(-255);
      while (regresando && flagUndo == 0) {
        updateMotorRamp();
        saveSdExt();
        while (digitalRead(LimSWT) == LOW) {
          updateMotorRamp();
        }
        regresando = false;
      }
      motorStop();
      controladoActivo = false;
      
      unsigned long inicioTimeout = millis();
      while (millis() - inicioTimeout < 6500) {
        motorSetTargetSpeed(255);
        updateMotorRamp();
      }
      motorStop();

      while(true && flagUndo == 0){  //CONFIRMACION PARA SALIR DEL MODO
        btnSelect.update();
        if (btnSelect.wasPressed()) {
          break;
        }
      }

      menu = 2;
      comms.sendEnterMenu();
      cambioModoTitulo();
      tft.print("Selector de modos");
      actualizarMenu();
      // actualizarMenu();
      flushBotones();
      break;
    }
  }
}

void esc_automatico() {
  flagUndo = 0;
  profundidad = 0;
  queDist = 2;
  regresando = false;
  Z = 0;

  cambioModoTitulo();
  tft.print("Escaneo Automatico");
  tft.setTextColor(ILI9341_WHITE, UI_NAVY);
  tft.setTextSize(2);
  tft.setCursor(45, 88);
  tft.print("Ingrese profundidad");

  tft.setTextSize(3);
  printPaddedTextF(tft, 110, 132,
                  UI_NAVY, ILI9341_WHITE,
                  1, 1, 0, 0,
                  "0.00 m");

  // --- Captura de profundidad ---
  // Equivalente a: while (digitalRead(selectButton) == LOW || profundidad < 0.11)
  
  nombreArchivoListo = crearCrudos();
  // Estado limpio por cada entrada al modo automático.
  //hayAgua = false;
  //hayTierra = false;
  //scanTerminadoDesdeSonda = true;

  flushBotones();
  
  while (true) {  //  ESPERA A QUE SE INGRESE PROFUNDIDAD

    // IMPORTANTÍSIMO: actualizar botones dentro del while (porque es bloqueante)
    btnSelect.update();
    btnUp.update();
    btnDown.update();
    btnUndo.update();
    processIncomingSerial();

    // Undo = salir (antes: if(digitalRead(undoButton)==HIGH){...})
    if (btnUndo.wasPressed()) {
      flagUndo = 1;
      break;
    }

    // Tu lógica de ajuste de profundidad
    // OJO: valorProf() ahora debe usar btnUp/btnDown (o seguir usando flags),
    // pero en cualquier caso ya actualizamos btnUp/btnDown arriba.
    valorProf();

    if (flagBoton == 1) {
      printPaddedTextF(tft, 110, 132,
                      UI_NAVY, ILI9341_WHITE,
                      1, 1, 0, 0,
                      "%.2f m",profundidad);
      flagBoton = 0;
    }

    // Select = confirmar SOLO si profundidad >= 0.11
    if (btnSelect.wasPressed() && profundidad >= 0.11) {
      //scanTerminadoDesdeSonda = false;          // Mover estas 2 lineas
      //comms.sendStartScan(); // comando 23      // al loop donde baja motor
      break;
    }
  }

  // Si NO se salió con undo, pasa a la pantalla de "Profundidad actual"
  //  APLICAR A MODO CONTROLADO DE AQUI...
  if (flagUndo == 0) {  //  SI NO SE PRESIONO UNDO EN EL MENU ANTERIOR CONTINUA A LA SIGUIENTE PANTALLA
    tft.fillRect(9, 52, 300, 177, UI_NAVY);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(2);
    tft.setCursor(xMenu, 88);
    tft.print("Profundidad actual:");
    tft.setTextSize(3);
    tft.setCursor(xMenu, 120);
    tft.print(" 0.00 / "); //  PONER LA PROFUNDIDAD INGRESADA
    tft.print(profundidad); //  PONER LA PROFUNDIDAD INGRESADA
    tft.print(" m"); //  PONER LA PROFUNDIDAD INGRESADA
    // Espera hasta que presionen Undo para regresar (antes: while(flagUndo==0 && digitalRead(undoButton)==LOW))
    
    //bool bloqueo = false;
    //while(!bloqueo){
      motorSetTargetSpeed(-255);
      while (digitalRead(LimSWT) == LOW) { // SUBIR EL MOTOR PARA EMPEZAR SIEMPRE HASTA ARRIBA
        //processIncomingSerial();
        updateMotorRamp();
      }
      //  DETENER EL MOTOR
      motorStop();
      //bool scan = true;
      scanTerminadoDesdeSonda = true;
      //comms.sendStartScan(); // comando 23
      int vueltas = 0;
      while(vueltas <= totalVueltasf || !scanTerminadoDesdeSonda){
        //  ESCANEAR BAJAR REPETIR
        btnSelect.update();
        baterryCharge();
      
        if(hayAgua){
          tft.setTextSize(2);
          printPaddedTextF(tft, xMenu, 88,
                          ILI9341_WHITE,UI_NAVY, 
                          1, 1, 0, 0,
                          "Agua detectada:    ");
          regresando = true;
          break;
        }else if(hayTierra){
          tft.setTextSize(2);
          printPaddedTextF(tft, xMenu, 88,
                          ILI9341_WHITE,UI_NAVY, 
                          1, 1, 0, 0,
                          "Tierra detectada:  ");
          regresando = true;
          break;
        }else if (btnSelect.wasPressed()) {
          tft.setTextSize(2);
          printPaddedTextF(tft, xMenu, 88,
                          ILI9341_WHITE,UI_NAVY, 
                          1, 1, 0, 0,
                        //"|||||||||||||||||||"
                          "Cancelacion manual:");
          comms.sendCancelSelect();
          regresando = true;
          break;
        }else if(scanTerminadoDesdeSonda == true){
          //  MOVER MOTOR HACIA ABAJO 3 CM
          //  LEER ENCODER Y ACTUALIZAR VUELTAS
          moverMotor3Cm();
          Z+=35;
          vueltas++;
          Serial.print("Vueltas: ");
          Serial.println(vueltas);
          float bajado = 0.035 * (vueltas + 1) + 0.03;
          tft.fillRect(xMenu, 120, 290, 25, UI_NAVY);
          tft.setTextColor(ILI9341_WHITE);
          tft.setTextSize(3);
          tft.setCursor(xMenu, 120); 
          tft.print(" "); 
          tft.print(bajado);
          tft.print(" / "); 
          tft.print(profundidad); 
          tft.print(" m");

          comms.sendLinkPing(); // comando 90
          linkPongRecibido = false;
          unsigned long inicioTimeout = millis();
          while (millis() - inicioTimeout < 1000) {
            processIncomingSerial();
          }
          if(!linkPongRecibido){
            tft.setTextSize(2);
            printPaddedTextF(tft, xMenu, 88,
                            ILI9341_WHITE,UI_NAVY, 
                            1, 1, 0, 0,
                            "Conexion fallida:  ");
            tft.fillRect(57, 4, 16, 16, UI_NAVY);
            tft.drawBitmap(57, 4, epd_bitmap_WARNING, 16, 16, ILI9341_WHITE);
            regresando = true;
            break;
          }
          comms.sendVl53StatusQuery(); // comando 53
          vl53NotReadyFlag = false;
          inicioTimeout = millis();
          while (millis() - inicioTimeout < 1000) {
            processIncomingSerial();
          }
          if(vl53NotReadyFlag){
            tft.setTextSize(2);
            printPaddedTextF(tft, xMenu, 88,
                            ILI9341_WHITE,UI_NAVY, 
                            1, 1, 0, 0,
                            "Sensor fallando:   ");
            tft.fillRect(35, 4, 16, 16, UI_NAVY);
            tft.drawBitmap(35, 4, epd_bitmap_WARNING, 16, 16, ILI9341_WHITE);
            regresando = true;
            break;
          }

          if(!hayAgua && !hayTierra){
            scanTerminadoDesdeSonda = false;
            comms.sendStartScan(); // comando 23
          }
        }else if(scanTerminadoDesdeSonda == false){
          // Solo intentamos guardar si la inicialización de archivo fue correcta.
          if (!nombreArchivoListo) {
            Serial.println("Advertencia: escaneo activo sin archivo de crudos inicializado.");
            delay(5);
            continue;
          }
          // Importante:
          // antes se abría el archivo en cada iteración y SOLO se cerraba cuando había lectura nueva,
          // lo que dejaba handles abiertos si no llegaba dato en ese ciclo.
          // Ahora abrimos/escribimos/cerramos únicamente cuando hay muestra disponible.
          if(hayNuevaLecturaVl53){
            FsFile crudo = sdInt.open(nombreArchivo, O_WRITE | O_APPEND | O_CREAT);
            if (!crudo) {
              Serial.print("Error al abrir archivo de crudos: ");
              Serial.println(nombreArchivo);
            } else {
              crudo.print(vl53TimeUs);
              crudo.print(" ");
              crudo.print(vl53DistMm);
              crudo.print(" ");
              crudo.println(Z);
              crudo.flush();
              crudo.close();
            }
            if(hayTiempoTotal){
              FsFile EscaneoTotales = sdInt.open(archivoCrudo, O_WRITE | O_APPEND | O_CREAT);
              EscaneoTotales.println(EscTotal);
              EscaneoTotales.flush();
              EscaneoTotales.close();
              hayTiempoTotal = false;
            }
            hayNuevaLecturaVl53 = false;
          }
        }
        processIncomingSerial();
      }
      regresando = true;
    //}
  }
  motorSetTargetSpeed(-255);
  while(regresando && flagUndo == 0){
    // Aqui mover el motor hacia arriba
    //saveSdExt();    INCLUIR SAVESDEXT EN VOID REGRESANDO?
    updateMotorRamp();
    saveSdExt();
    while(digitalRead(LimSWT) == LOW){  
      updateMotorRamp();
    }
    regresando = false;
  }
  //  DETENER EL MOTOR
  motorStop();
  
  unsigned long inicioTimeout = millis();
  while (millis() - inicioTimeout < 6500) {
    motorSetTargetSpeed(255);
    updateMotorRamp();
  }
  motorStop();

  while(true && flagUndo == 0){  //CONFIRMACION PARA SALIR DEL MODO
    btnSelect.update();
    if (btnSelect.wasPressed()) {
      break;
    }
  }
  //  HASTA ACA

  // Regresar a selector de modos
  menu = 3;
  comms.sendEnterMenu();
  cambioModoTitulo();
  tft.print("Selector de modos");
  actualizarMenu();
}
void prueba_sistema(){
  // Arranca con velocidad BAJA (muchos módulos fallan a 25MHz)
  SdSpiConfig cfg(SD_CS, DEDICATED_SPI, SD_SCK_MHZ(SD_MHZ), &SPI1);
  cambioModoTitulo();
  tft.print("Prueba de sistema");
  tft.setTextSize(5);

  printPaddedTextF(tft, 57, 111,
                  UI_NAVY,ILI9341_WHITE,
                  1, 1, 0, 0,
                  "Iniciar");
  flushBotones();
  bool sdOk = false;
  uint32_t lastSdTry = 0;
  uint32_t lastVl53Query = 0;
  uint32_t lastPing = 0;

  const uint32_t SD_RETRY_MS = 300;
  const uint32_t VL53_QUERY_MS = 200;
  const uint32_t PING_MS = 250;
  while(true){
    btnSelect.update();
    baterryCharge();
    btnUndo.update();
    processIncomingSerial();

    if (btnUndo.wasPressed()) break;

    if (btnSelect.wasPressed()){
      tft.fillRect(9, 52, 300, 177, UI_NAVY);
      tft.setTextSize(3);
      tft.setTextColor(ILI9341_WHITE);
      tft.setCursor(xMenu, 111);
      tft.print("Ejecutando");
      const int barX = 35;
      const int barY = 150;
      const int barW = 250;
      const int barH = 18;
      // Contorno de la barra
      tft.drawRect(barX, barY, barW, barH, ILI9341_WHITE);
      // Interior vacío
      tft.fillRect(barX + 1, barY + 1, barW - 2, barH - 2, UI_NAVY);
      int lastFillW = -1;
      int lastPercent = -1;

      const uint32_t TEST_TOTAL_MS = 3000;
      const uint32_t HMI_DIR_MS = 1500;
      const uint32_t SCAN_TIMEOUT_MS = 25000; //  AJUSTAR
      const int HOLD_PERCENT = 80;

      uint32_t motorElapsed = 0;
      bool motorPausedPorScan = false;
      bool timeoutScan = false;

      scanTerminadoDesdeSonda = false;
      comms.sendStartScan();
      motorSetSpeed(145);

      uint32_t startTime = millis();
      uint32_t lastStepTime = startTime;
      while (motorElapsed < TEST_TOTAL_MS) {
        const uint32_t now = millis();
        uint32_t dt = now - lastStepTime;
        lastStepTime = now;

        if (!motorPausedPorScan && dt > 0) {
          motorElapsed += dt;
          if (motorElapsed > TEST_TOTAL_MS) motorElapsed = TEST_TOTAL_MS;
        }
        
        // 1) SD: reintento espaciado
        if (!sdOk && (now - lastSdTry >= SD_RETRY_MS)) {
          lastSdTry = now;
          if (sdExt.begin(cfg) && sdInt.begin(SdioConfig(FIFO_SDIO))) { //   BLOQUEANTE
            sdOk = true;
          }
        }

        // 2) VL53: consulta espaciada
        if (!vl53ReadyFlag && (now - lastVl53Query >= VL53_QUERY_MS)) {
          lastVl53Query = now;
          comms.sendVl53StatusQuery(); // CMD 53
        }

        // 3) Ping de enlace (independiente de vl53Ok)
        if (!linkPongRecibido && (now - lastPing >= PING_MS)) {
          lastPing = now;
          comms.sendLinkPing(); // CMD 90
        }

        // 4) Leer payloads decodificados y filtrar respuesta esperada.
        processIncomingSerial();
        updateMotorRamp();
        
        int porcentaje = (motorElapsed * 100) / TEST_TOTAL_MS;

        // Si llega al 80% y no termina scan de sonda, pausar hasta recibir CMD_SCAN_FINISHED.
        if (!scanTerminadoDesdeSonda && porcentaje >= HOLD_PERCENT) {
          motorPausedPorScan = true;
          porcentaje = HOLD_PERCENT;
          //motorStop();

          if (now - startTime >= SCAN_TIMEOUT_MS) {
            timeoutScan = true;
            break;
          }
        } else {
          motorPausedPorScan = false;
          
        }
        if (motorElapsed < HMI_DIR_MS) {
            // Primeros 1.5s: hacia abajo.
            motorSetTargetSpeed(255);
          } else if (motorElapsed < TEST_TOTAL_MS) {
            // Siguientes 1.5s: hacia arriba.
            motorSetTargetSpeed(-255);
          }
        motorStop();

        int fillW = (porcentaje * (barW - 2)) / 100;

        // Actualizar barra solo si cambió el ancho
        if (fillW != lastFillW) {
          tft.fillRect(barX + 1, barY + 1, fillW, barH - 2, ILI9341_WHITE);
          lastFillW = fillW;
        }

        // Actualizar porcentaje solo si cambió
        if (porcentaje != lastPercent) {
          tft.fillRect(130, 170, 60, 28, UI_NAVY);  // limpia texto anterior
          tft.setCursor(130, 170);
          tft.print(porcentaje);
          tft.print(" %");
          lastPercent = porcentaje;
        }
        delay(2); // cede CPU y evita bucle agresivo
      }
      
      motorStop();
      if (timeoutScan) {
        Serial.println("[PRUEBA] Timeout esperando CMD_SCAN_FINISHED");
      }
      
      // Dibujo de bitmaps
        tft.fillRect(13, 4, 75, 16, UI_NAVY);
        if(sdOk){
          tft.drawBitmap(13, 4, epd_bitmap_SD,     16, 16, ILI9341_WHITE);
        }else{
          tft.drawBitmap(13, 4, epd_bitmap_WARNING, 16, 16, ILI9341_WHITE);
        }
        if(vl53ReadyFlag){
          tft.drawBitmap(35, 4, epd_bitmap_Sensor, 16, 16, ILI9341_WHITE);
        }else{
          tft.drawBitmap(35, 4, epd_bitmap_WARNING, 16, 16, ILI9341_WHITE);
        }
        if(linkPongRecibido){
          tft.drawBitmap(57, 4, epd_bitmap_Enlace, 16, 16, ILI9341_WHITE);
        }else{
          tft.drawBitmap(57, 4, epd_bitmap_WARNING, 16, 16, ILI9341_WHITE);
        }
      //*/
      // AGREGAR MOVIMIENTO DE MOTORES
      tft.fillRect(xMenu, 111, 280, 100, UI_NAVY);
      tft.setTextSize(5);
      printPaddedTextF(tft, 57, 111,
                      UI_NAVY,ILI9341_WHITE,
                      1, 1, 0, 0,
                      "Iniciar");
    }
  }

  menu = 4;
  comms.sendEnterMenu();
  cambioModoTitulo();
  tft.print("Selector de modos");
  actualizarMenu();
}
///////////////////////////////////////   Funciones repetitivas
void cambioModoTitulo(){
  tft.fillRect(13, 30, 221, 16, UI_NAVY);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(13,30);
  tft.fillRect(9, 52, 300, 177, UI_NAVY);
}

void valorProf(){

  static bool holdModeUp = false;
  static bool holdModeDown = false;

  // -------- BOTON UP --------
  if (btnUp.wasPressed()) {
    // Primer toque siempre suma 0.10
    profundidad += 0.10;
    flagBoton = 1;
    holdModeUp = false;
  }

  if (btnUp.isPressed()) {

    // Si ya pasó el tiempo de hold,
    // usamos repetición para sumar 1
    if (btnUp.isRepeating()) {

      if (!holdModeUp) {
        // La primera repetición ocurre justo al inicio,
        // la ignoramos porque ya sumamos 0.10 arriba
        holdModeUp = true;
      } else {
        profundidad += 1.0;
        flagBoton = 1;
      }
    }

  } else {
    holdModeUp = false;
  }


  // -------- BOTON DOWN --------
  if (btnDown.wasPressed()) {
    profundidad -= 0.10;
    if (profundidad < 0) profundidad = 0;
    flagBoton = 1;
    holdModeDown = false;
  }

  if (btnDown.isPressed()) {

    if (btnDown.isRepeating()) {

      if (!holdModeDown) {
        holdModeDown = true;
      } else {
        profundidad -= 1.0;
        if (profundidad < 0) profundidad = 0;
        flagBoton = 1;
      }
    }

  } else {
    holdModeDown = false;
  }

  // -------- CALCULOS EXISTENTES --------
  if(queDist==1){
    distInicio = profundidad;
    inicioControlado = (int)((profundidad - 0.03) / 0.035) - 1;
  }else if(queDist==2){
    distFinal = profundidad;
    totalVueltasf = (int)((profundidad - 0.03) / 0.035) - 1;
  }
}

void flushBotones(){
  while (true) {
    btnSelect.update(); btnUp.update(); btnDown.update(); btnUndo.update();
    processIncomingSerial();
    if (!btnSelect.isPressed() && !btnUp.isPressed() && !btnDown.isPressed() && !btnUndo.isPressed()) break;
  }
}

bool readIncomingPayload(SubTerraComms::DecodedPayload& payload){
  SubTerraComms::ParseResult parseStatus = SubTerraComms::PARSE_INCOMPLETE;
  return comms.pollPayload(payload, &parseStatus);
}

void applyPayloadEffects(const SubTerraComms::DecodedPayload& payload){
  // Guardar siempre el último payload procesado
  lastPayloadCmd = payload.cmd;
  lastPayloadHasValue = payload.hasValue;
  lastPayloadValue = payload.value;
  lastPayloadHasTimestamp = payload.hasTimestamp;
  lastPayloadTimestampUs = payload.timestampUs;

  switch (payload.cmd) {
    case SubTerraComms::CMD_SCAN_FINISHED:
      scanTerminadoDesdeSonda = true;
      Serial.println("[MENU] RX CMD 32 -> Escaneo terminado");
      break;

    case SubTerraComms::CMD_AGUA_DETECT:
      hayAgua = true;
      Serial.println("[MENU] RX CMD 33 -> Agua detectada");
      break;

    case SubTerraComms::CMD_TIERRA_DETECT:
      hayTierra = true;
      Serial.println("[MENU] RX CMD 34 -> Tierra detectada");
      break;

    case SubTerraComms::CMD_VL53_READY:
      vl53ReadyFlag = true;
      Serial.println("[MENU] RX CMD 35 -> VL53 listo");
      break;

    case SubTerraComms::CMD_VL53_NOT_READY:
      vl53NotReadyFlag = true;
      Serial.println("[MENU] RX CMD 36 -> VL53 no listo");
      break;

    case SubTerraComms::CMD_VL53_READING:
      if (payload.hasValue && payload.hasTimestamp) {
        vl53DistMm = payload.value;
        vl53TimeUs = payload.timestampUs - EscInicio;
        hayNuevaLecturaVl53 = true;
        Serial.print("[MENU] RX CMD 54 -> VL53 reading: ");Serial.print(vl53DistMm);Serial.print(", ");Serial.println(vl53TimeUs);
      }
      break;

    case SubTerraComms::CMD_LINK_PONG:
      linkPongRecibido = true;
      Serial.println("[MENU] RX CMD 91 -> Link pong");
      break;

    case SubTerraComms::CMD_TIME_START:
      if (payload.hasTimestamp) {
        EscInicio = payload.timestampUs;
        hayTiempoInicio = true;
        Serial.print("[MENU] RX CMD 14 -> Time start(us): ");
        Serial.println((unsigned long)EscInicio);
      }
      break;

    case SubTerraComms::CMD_TIME_FINISH:
      if (payload.hasTimestamp) {
        EscTotal = payload.timestampUs - EscInicio;
        hayTiempoTotal = true;
        Serial.print("[MENU] RX CMD 41 -> Time total Final(us): ");
        Serial.println((unsigned long)EscTotal);
      }
      break;

    default:
      Serial.printf("[MENU] RX CMD %u\n", (unsigned)payload.cmd);
      break;
  }
}

void processIncomingSerial(){
  SubTerraComms::DecodedPayload payload;
  while (readIncomingPayload(payload)) {
    applyPayloadEffects(payload);
  }
}

void moverMotor3Cm(){
  newPosition = 0;
  myEnc.write(0);
  //int distConstante = 3000;
  // Revisar que la funcion .read() no entregue valores 
  motorSetTargetSpeed(255);
  while(newPosition <= dist3Cm/2 && !hayAgua && !hayTierra) {
    processIncomingSerial();
    btnUp.update();
    btnDown.update();
    updateMotorRamp();
    newPosition = myEnc.read();
  }
  if(!hayAgua && !hayTierra){
    motorSetTargetSpeed(50);
  }
  while(newPosition <= dist3Cm && !hayAgua && !hayTierra) {
    processIncomingSerial();
    btnUp.update();
    btnDown.update();
    updateMotorRamp();
    newPosition = myEnc.read();
  }
  // Frenar al llegar a la distancia.
  motorBrake();
  myEnc.write(0);
    //Serial.println(newPosition);
}

void moverMotor3CmArriba(){
  newPosition = 0;
  myEnc.write(0);
  motorSetTargetSpeed(-255);
  while (abs(newPosition) <= dist3Cm/2 && !hayAgua && !hayTierra) {
    processIncomingSerial();
    btnUp.update();
    btnDown.update();
    updateMotorRamp();
    if (digitalRead(LimSWT) == HIGH) {
      break;
    }
    newPosition = abs(myEnc.read());
  }
  if (!hayAgua && !hayTierra) {
    motorSetTargetSpeed(-50);
  }
  while (abs(newPosition) <= dist3Cm && !hayAgua && !hayTierra) {
    processIncomingSerial();
    btnUp.update();
    btnDown.update();
    updateMotorRamp();
    if (digitalRead(LimSWT) == HIGH) {
      break;
    }
    newPosition = abs(myEnc.read());
  }
  motorBrake();
  myEnc.write(0);
}

void saveSdExt(){
  //Procesar datos crudos y guardar en archivo final ---
  bool archivoGuardado = false;
  FsFile EscaneoTotales = sdInt.open(archivoCrudo, O_RDONLY);
  if(EscaneoTotales){
    FsFile crudoRead = sdInt.open(nombreArchivo, O_RDONLY);
    if(crudoRead){
      FsFile Archivo = sdExt.open(nombreArchivo, O_WRONLY | O_CREAT | O_APPEND);
      if(Archivo){
        unsigned int lastAltura = 0;
        unsigned long escTotal1 = 1;
        while(crudoRead.available()){
          updateMotorRamp();
          if(digitalRead(LimSWT) == HIGH){
            motorStop();
            regresando = false;
          }
          String line = crudoRead.readStringUntil('\n');
          unsigned long t;
          int dist;
          unsigned int altura;
          
          if(sscanf(line.c_str(), "%lu %d %u", &t, &dist, &altura) == 3){//     AGREGAR LECTURA DE Z
            if(altura != lastAltura){
              lastAltura = altura;
              String tiemposfinales = EscaneoTotales.readStringUntil('\n');
              if(sscanf(tiemposfinales.c_str(), "%lu", &escTotal1) == 1){
                if(escTotal1 == 0){
                  escTotal1 = 1;
                }
              }
            }
            Serial.println(escTotal1);
            // ángulo interpolado entre tiempo inicial y final
            float angle = ((float)(t) / (float)escTotal1) * 2.0 * PI;
            float X = originX + dist * cos(angle);
            float Y = originY + dist * sin(angle);
            Archivo.print(X); Archivo.print(" ");
            Archivo.print(Y); Archivo.print(" ");
            Archivo.println(altura);
          }
        }
        Archivo.flush();
        Archivo.close();
        archivoGuardado = true;
        //SD.remove(archivoCrudo); // borrar crudo
        EscaneoTotales.close();
        sdInt.remove(archivoCrudo);   // o el nombre real que quieras borrar
      }else {
        Serial.println(F("Error abriendo archivo de salida en SD externa"));
        EscaneoTotales.close();
      }
      crudoRead.close();
      if (archivoGuardado && !respaldarSdInterna()) {
        Serial.println(F("Error respaldando archivo final en SD interna"));
      }
    } else {
      Serial.println(F("Error leyendo crudo.txt"));
      EscaneoTotales.close();
    }
  } else {
    finalDatos = true;
    Serial.println(F("Error leyendo TempoTimeF.txt"));
    //EscaneoTotales.close();
  }
  // ---------------------------------------------------------
}

bool respaldarSdInterna(){
  if (nombreArchivo[0] == '\0') {
    Serial.println(F("No hay nombre de archivo listo para respaldar."));
    return false;
  }

  FsFile Archivo = sdExt.open(nombreArchivo, O_RDONLY);
  if (!Archivo) {
    Serial.print(F("Error abriendo archivo de respaldo en SD externa: "));
    Serial.println(nombreArchivo);
    return false;
  }

  FsFile copiaInterna = sdInt.open(nombreArchivo, O_WRONLY | O_CREAT | O_TRUNC);
  if (!copiaInterna) {
    Serial.print(F("Error creando copia en SD interna: "));
    Serial.println(nombreArchivo);
    Archivo.close();
    return false;
  }

  uint8_t buffer[512];
  bool copiaCompleta = true;

  while (true) {
    int leidos = Archivo.read(buffer, sizeof(buffer));
    if (leidos < 0) {
      Serial.println(F("Error leyendo archivo desde SD externa."));
      copiaCompleta = false;
      break;
    }
    if (leidos == 0) {
      break;
    }

    size_t escritos = copiaInterna.write(buffer, (size_t)leidos);
    if (escritos != (size_t)leidos) {
      Serial.println(F("Error escribiendo copia en SD interna."));
      copiaCompleta = false;
      break;
    }

    processIncomingSerial();
    updateMotorRamp();
  }

  copiaInterna.flush();

  Archivo.close();
  copiaInterna.close();

  if (!copiaCompleta) {
    sdInt.remove(nombreArchivo);
    return false;
  }

  nombreArchivoListo = true;
  Serial.print(F("Respaldo en SD interna completado: "));
  Serial.println(nombreArchivo);
  return true;
}

bool crearCrudos(){
  // Pedimos a la función que decida qué archivo usar
  if (!obtenerNombreArchivo(nombreArchivo, sizeof(nombreArchivo))) {
    Serial.println("No se pudo determinar el archivo a usar.");
    return false;
  }

  // Abrimos/cerramos de inmediato para asegurar que el archivo quede listo.
  // Nota:
  // - Si obtenerNombreArchivo() decidió REUTILIZAR uno existente, aquí solo lo tocamos.
  // - Si decidió CREAR uno nuevo, O_CREAT lo crea en este momento.
  FsFile nuevoCrudo = sdInt.open(nombreArchivo, O_WRITE | O_APPEND | O_CREAT);
  if (!nuevoCrudo) {
    Serial.print("No se pudo preparar el archivo de crudos: ");
    Serial.println(nombreArchivo);
    return false;
  }
  nuevoCrudo.close();

  // Mostramos el nombre elegido
  Serial.print("Archivo seleccionado: ");
  Serial.println(nombreArchivo);
  return true;
}

void baterryCharge(){
  int batteryPct = readBatteryCharge();
  if(batteryPct == 100 && !cienPct){
    tft.fillRect(275, 4, 32, 16, UI_NAVY);
    tft.drawBitmap(275, 4, epd_bitmap_bateriaFull32, 32, 16, ILI9341_WHITE);
    cienPct = true;
    tresPct = false;
    dosPct  = false;
    unoPct  = false;
    zeroPct = false;
  }else if(batteryPct == 75 && !tresPct){
    tft.fillRect(275, 4, 32, 16, UI_NAVY);
    tft.drawBitmap(275, 4, epd_bitmap_bateriaTres32, 32, 16, ILI9341_WHITE);
    cienPct = false;
    tresPct = true;
    dosPct  = false;
    unoPct  = false;
    zeroPct = false;
  }else if(batteryPct == 50 && !dosPct){
    tft.fillRect(275, 4, 32, 16, UI_NAVY);
    tft.drawBitmap(275, 4, epd_bitmap_BateriaDos32, 32, 16, ILI9341_WHITE);
    cienPct = false;
    tresPct = false;
    dosPct  = true;
    unoPct  = false;
    zeroPct = false;
  }else if(batteryPct == 25 && !unoPct){
    tft.fillRect(275, 4, 32, 16, UI_NAVY);
    tft.drawBitmap(275, 4, epd_bitmap_bateriaUno32, 32, 16, ILI9341_WHITE);
    cienPct = false;
    tresPct = false;
    dosPct  = false;
    unoPct  = true;
    zeroPct = false;
  }else if(batteryPct == 0 && !zeroPct){
    tft.fillRect(275, 4, 32, 16, UI_NAVY);
    tft.drawBitmap(275, 4, epd_bitmap_bateria32, 32, 16, ILI9341_WHITE);
    cienPct = false;
    tresPct = false;
    dosPct  = false;
    unoPct  = false;
    zeroPct = true;
  }
}
// -----------------------------------------------------------------------------
// Esta función decide qué archivo usar siguiendo este formato:
  // ESC0.txt
  // ESC1.txt
  // ESC2.txt
  // ...
  //
  // Lógica:
  // 1. Busca el primer índice que NO existe en la secuencia ESC<N>.txt.
  // 2. Identifica el último archivo existente (ESC i-1).
  // 3. Si el último archivo tiene MÁS de 12 bytes -> devuelve un nombre nuevo (ESC i).
  // 4. Si el último archivo tiene 12 bytes o MENOS -> lo trunca y reutiliza (ESC i-1).
  //
  // Nota de diseño:
  // Así se evita llenar la SD con archivos "vacíos" o casi vacíos, pero se mantiene
  // la secuencia cuando el último archivo sí contiene datos reales.
  //
  // Parámetros:
  // - nombreFinal: buffer donde se guardará el nombre resultante
  // - len: tamaño de ese buffer
  //
  // Retorna:
  // - true si todo salió bien
  // - false si hubo error
// -----------------------------------------------------------------------------
bool obtenerNombreArchivo(char* nombreFinal, size_t len){
  // "i" va a recorrer ESC0.txt, ESC1.txt, ESC2.txt, ...
  int i = 0;

  // Buffer temporal para construir nombres de archivo
  char nombreActual[32];

  // ---------------------------------------------------------------------------
  // PASO 1: Buscar el primer archivo que NO exista.
  //
  // Ejemplo:
  // si existen ESC0, ESC1, ESC2 y no existe ESC3:
  // el loop terminará con i = 3
  //
  // Entonces:
  // - el siguiente nombre nuevo sería ESC3.txt
  // - el último archivo existente sería ESC2.txt
  // ---------------------------------------------------------------------------
  while (true) {
    // Construye el nombre: ESC0.txt, ESC1.txt, ESC2.txt, etc.
    snprintf(nombreActual, sizeof(nombreActual), "ESC%d.txt", i);

    // Si el archivo existe, seguimos buscando el siguiente
    if (sdInt.exists(nombreActual)) {
      i++;
    } 
    // Si NO existe, ya encontramos el hueco y salimos
    else {
      break;
    }
  }

  
  // Si no existe ningún archivo todavía, arrancamos en ESC0.
  if (i == 0) {
    int chars = snprintf(nombreFinal, len, "ESC0.txt");
    if (chars < 0 || (size_t)chars >= len) {
      Serial.println("Error: buffer insuficiente para guardar nombre de archivo.");
      return false;
    }
    return true;
  }

  // Existe al menos un archivo. Revisamos el último archivo existente.
  char ultimoNombre[32];
  int charsUltimo = snprintf(ultimoNombre, sizeof(ultimoNombre), "ESC%d.txt", i - 1);
  if (charsUltimo < 0 || (size_t)charsUltimo >= sizeof(ultimoNombre)) {
    Serial.println("Error: nombre del ultimo archivo excede el buffer.");
    return false;
  }

  // ---------------------------------------------------------------------------
  // PASO 3: Abrir el último archivo para revisar su tamaño
  // ---------------------------------------------------------------------------
  FsFile ultimoArchivo = sdInt.open(ultimoNombre, O_RDONLY);

  // Si no se pudo abrir, regresamos error
  if (!ultimoArchivo) {
    Serial.println("Error: no se pudo abrir el ultimo archivo para revisar su tamano.");
    return false;
  }

  // Obtenemos el tamaño en bytes
  uint32_t tamanoUltimo = ultimoArchivo.size();

  // Cerramos el archivo porque ya no lo necesitamos abierto
  ultimoArchivo.close();

  // ---------------------------------------------------------------------------
  // PASO 4: Decidir si reutilizamos el último archivo o creamos uno nuevo
  // Regla solicitada:
  // - Si el último tiene MÁS de 12 bytes -> crear siguiente archivo.
  // - Si tiene 12 bytes o menos -> truncar y reutilizar último.
  if (tamanoUltimo > 12) {
    int chars = snprintf(nombreFinal, len, "ESC%d.txt", i);
    if (chars < 0 || (size_t)chars >= len) {
      Serial.println("Error: buffer insuficiente para guardar nombre de archivo.");
      return false;
    }
    return true;
  }
  // Reutilización: se limpia el contenido del último archivo antes de usarlo.
  FsFile truncFile = sdInt.open(ultimoNombre, O_WRITE | O_TRUNC);
  if (!truncFile) {
    Serial.println("Error: no se pudo truncar el ultimo archivo para reutilizarlo.");
    return false;
  }
  truncFile.close();
  int chars = snprintf(nombreFinal, len, "%s", ultimoNombre);
  if (chars < 0 || (size_t)chars >= len) {
    Serial.println("Error: buffer insuficiente para guardar nombre de archivo.");
    return false;
  }

  return true;
}

// --------------------------------------------------
// Inicialización Motor
  // --------------------------------------------------
  void motorBegin() {
    pinMode(PIN_PWMA, OUTPUT);
    pinMode(PIN_AIN1, OUTPUT);
    pinMode(PIN_AIN2, OUTPUT);
    //pinMode(PIN_STBY, OUTPUT);
    // En Teensy, subir frecuencia PWM evita pitido audible del motor/driver.
    analogWriteResolution(8);
    analogWriteFrequency(PIN_PWMA, 20000);

    //digitalWrite(PIN_STBY, HIGH); // activar driver
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
    }else {
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
    //digitalWrite(PIN_STBY, LOW);
  }

  // Sacar el driver de standby
  void motorWake() {
    //digitalWrite(PIN_STBY, HIGH);
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
      //Serial.println(newPosition);
      //Serial.print("SPEED: ");
      //Serial.println(currentSpeed);
      if (currentSpeed > targetSpeed) currentSpeed = targetSpeed;
      applyMotorSpeed(currentSpeed);
    }
    else if (currentSpeed > targetSpeed) {
      currentSpeed -= rampStep;
      //Serial.println(newPosition);
      //Serial.print("SPEED: ");
      //Serial.println(currentSpeed);
      if (currentSpeed < targetSpeed) currentSpeed = targetSpeed;
      applyMotorSpeed(currentSpeed);
    }
  }
//
