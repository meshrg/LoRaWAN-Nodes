#include <Adafruit_SSD1306.h>
#include <splash.h>

#include <Arduino.h>

//Libraries for LoRa
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

//Libraries for OLED Display
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//define the pins used by the LoRa transceiver module
#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     14   // GPIO14 -- SX1278's RESET
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define BAND    915E6

//OLED pins
#define OLED_SDA 4
#define OLED_SCL 15
#define OLED_RST 16
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
//DISPLAY
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

//
// For normal use, we require that you edit the sketch to replace FILLMEIN
// with values assigned by the TTN console. However, for regression tests,
// we want to be able to compile these scripts. The regression tests define
// COMPILE_REGRESSION_TEST, and in that case we define FILLMEIN to a non-
// working but innocuous value.
//
#ifdef COMPILE_REGRESSION_TEST
# define FILLMEIN 0
#else
# warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
# define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
#endif
////////////////////////////////////////// AQUÍ VAN LOS PINES Y VARIABLES DE LOS SENSORES

#define LED_BUILTIN 13
#define SENSOR  12

long currentMillis = 0;
long previousMillis = 0;
int interval = 1000;
boolean ledState = LOW;
float calibrationFactor = 4.5;
volatile byte pulseCount;
byte pulse1Sec = 0;
float flowRate;
unsigned int flowMilliLitres;
unsigned long totalMilliLitres;
unsigned long Litres;
float totalLitres;


//contador de msj LoRa enviados
int counter = 0;
///////////////////////////////////////////FIN DE AQUÍ VAN LOS PINES Y VARIABLES DE LOS SENSORES
void do_send(osjob_t* j);

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

void os_getArtEui (u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = { 0x03, 0xFC, 0xF6, 0x85, 0x99, 0x02, 0x72, 0x67 };

void os_getDevEui (u1_t* buf) {
  memcpy_P(buf, DEVEUI, 8);
}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0x52, 0xE5, 0x4E, 0x35, 0xB9, 0x1D, 0x36, 0xAA, 0xEA, 0x44, 0x98, 0xDB, 0xFB, 0x8B, 0xB9, 0x03 };


void os_getDevKey (u1_t* buf) {
  memcpy_P(buf, APPKEY, 16);
}


///////-------------------------------INICIO PARA CAYENNE-----------------------
//LPP_data     0     1     2     3     4     5     6     7     8     9     10    11    12
static uint8_t mydata[4] = {0x01, 0x66, 0x00, 0x00}; //0xO1,0x02,0x03,0x04 is Data Channel,0x67,0x68,0x01,0x01 is Data Type
//opencml 0     1     2     3          closecml  0     1     2     3
//static uint8_t opencml[4] = {0x03, 0x00, 0x64, 0xFF}, closecml[4] = {0x03, 0x00, 0x00, 0xFF}; //the payload of the cayenne or ttn downlink

////////////----------------------------- FIN PARA CAYENNE-----------------------


//static uint8_t mydata[] = "Hello, world!";
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 14,
  .dio = {/*dio0*/ 26, /*dio1*/ 35, /*dio2*/ 34}
};



void printHex2(unsigned v) {
  v &= 0xff;
  if (v < 16)
    Serial.print('0');
  Serial.print(v, HEX);
}

void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      {
        u4_t netid = 0;
        devaddr_t devaddr = 0;
        u1_t nwkKey[16];
        u1_t artKey[16];
        LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
        Serial.print("netid: ");
        Serial.println(netid, DEC);
        Serial.print("devaddr: ");
        Serial.println(devaddr, HEX);
        Serial.print("AppSKey: ");
        for (size_t i = 0; i < sizeof(artKey); ++i) {
          if (i != 0)
            Serial.print("-");
          printHex2(artKey[i]);
        }
        Serial.println("");
        Serial.print("NwkSKey: ");
        for (size_t i = 0; i < sizeof(nwkKey); ++i) {
          if (i != 0)
            Serial.print("-");
          printHex2(nwkKey[i]);
        }
        Serial.println();
      }
      // Disable link check validation (automatically enabled
      // during join, but because slow data rates change max TX
      // size, we don't use it in this example.
      LMIC_setLinkCheckMode(0);
      break;
    /*
      || This event is defined but not used in the code. No
      || point in wasting codespace on it.
      ||
      || case EV_RFU1:
      ||     Serial.println(F("EV_RFU1"));
      ||     break;
    */
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.print(F("Received "));
        Serial.print(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    /*
      || This event is defined but not used in the code. No
      || point in wasting codespace on it.
      ||
      || case EV_SCAN_FOUND:
      ||    Serial.println(F("EV_SCAN_FOUND"));
      ||    break;
    */
    case EV_TXSTART:
      Serial.println(F("EV_TXSTART"));
      break;
    case EV_TXCANCELED:
      Serial.println(F("EV_TXCANCELED"));
      break;
    case EV_RXSTART:
      /* do not print anything -- it wrecks timing */
      break;
    case EV_JOIN_TXCOMPLETE:
      Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
      break;

    default:
      Serial.print(F("Unknown event: "));
      Serial.println((unsigned) ev);
      break;
  }
}


void do_send(osjob_t* j) {
  //----------scope variables-----------------------------



  ///----------fin scope variables

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {


    ////////////////////////////////////////captura de datos del sensor



    currentMillis = millis();
    if (currentMillis - previousMillis > interval) {

      pulse1Sec = pulseCount;
      pulseCount = 0;

      // Because this loop may not complete in exactly 1 second intervals we calculate
      // the number of milliseconds that have passed since the last execution and use
      // that to scale the output. We also apply the calibrationFactor to scale the output
      // based on the number of pulses per second per units of measure (litres/minute in
      // this case) coming from the sensor.
      flowRate = ((1000.0 / (millis() - previousMillis)) * pulse1Sec) / calibrationFactor;
      previousMillis = millis();

      // Divide the flow rate in litres/minute by 60 to determine how many litres have
      // passed through the sensor in this 1 second interval, then multiply by 1000 to
      // convert to millilitres.
      flowMilliLitres = (flowRate / 60) * 1000;

      // Add the millilitres passed in this second to the cumulative total
      totalMilliLitres += flowMilliLitres;
      //total litros
      Litres=totalMilliLitres/1000;
      // Print the flow rate for this second in litres / minute
      Serial.print("Caudal: ");
      Serial.print(flowRate,3);  
      Serial.print(" L/min");
      Serial.print("\t");       // Print tab space

      // Print the cumulative total of litres flowed since starting
      Serial.print("Cantidad de líquido medida: ");
      Serial.print(totalMilliLitres);
      Serial.print(" mL / ");
      Serial.print(Litres);
      Serial.println(" L");
    }

    totalLitres=float(totalMilliLitres);//Cambiar a "float(Litres)" para enviar el total de litros
    byte dataArray[4] = {
      ((uint8_t*)&totalLitres)[0],
      ((uint8_t*)&totalLitres)[1],
      ((uint8_t*)&totalLitres)[2],
      ((uint8_t*)&totalLitres)[3]
    };


   
    float b;
      ((uint8_t*)&b)[0] = dataArray[0];
      ((uint8_t*)&b)[1] = dataArray[1];
      ((uint8_t*)&b)[2] = dataArray[2];
      ((uint8_t*)&b)[3] = dataArray[3];
       Serial.print("variable miliLitres: ");
      Serial.print (totalLitres, 3);
      Serial.println ("mL");
      Serial.print("variable b: ");
      Serial.print (b, 3);
      Serial.println ("mL");

      Serial.print("dataArray: "); 
      for (int i = 0; i < 3; i++) {
              Serial.print(dataArray[i]);
            //mydata[i] = dataArray[i];
          }


  //Envío del paquete LoRaWAN
    LMIC_setTxData2(1, dataArray, sizeof(dataArray), 0);
    Serial.println(F("Packet queued"));
    counter++;

  }

  /////////////////////////////////////// FIN DE captura de datos del sensor

  /////////////////////// DISPLAY comandos para mostrar en pantalla
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 20);
  display.setTextSize(1);
  display.print("Caudal: ");
  //int b=(int)a;
  display.print(flowRate);
  display.print(" L/min");
  display.setCursor(0, 30);
  display.print("Total mL: ");
  display.print(totalMilliLitres);
  display.print(" mL");
  display.setCursor(0, 40);
  display.print("Total Litros: ");
  display.print(Litres);
  display.print(" L");
  display.setCursor(0,50);
  display.print("Contador msj: ");
  display.print(counter); 
  display.display();
  ///////////////////////////// fin de comandos para mostrar en pantalla

/////////////DECODER EN TTS
 /*
       usse The Follow Decoder Function on the Things Statck
       https://www.thethingsnetwork.org/forum/t/decode-float-sent-by-lopy-as-node/8757/2
       function Decoder(bytes, port) {

      // Based on https://stackoverflow.com/a/37471538 by Ilya Bursov
      function bytesToFloat(bytes) {
        // JavaScript bitwise operators yield a 32 bits integer, not a float.
        // Assume LSB (least significant byte first).
        var bits = bytes[3]<<24 | bytes[2]<<16 | bytes[1]<<8 | bytes[0];
        var sign = (bits>>>31 === 0) ? 1.0 : -1.0;
        var e = bits>>>23 & 0xff;
        var m = (e === 0) ? (bits & 0x7fffff)<<1 : (bits & 0x7fffff) | 0x800000;
        var f = sign * m * Math.pow(2, e - 150);
        return f;
      }

      // Test with 0082d241 for 26.3134765625
      return {
        // Take bytes 0 to 4 (not including), and convert to float:
        temp: bytesToFloat(bytes.slice(0, 4)).toFixed(3)
      };
      }


    */
//////////////
}
// Next TX is scheduled after TX_COMPLETE event.


//////////////////////////////////////////////////////AQUÍ VAN LAS FUNCIONES DEL SENSOR
void IRAM_ATTR pulseCounter()
{
  pulseCount++;
}

/////////////////////////////////////////////////FIN DE FUNCIONES DEL SENSOR
void setup() {
  Serial.begin(115200);
  Serial.println(F("Starting"));

#ifdef VCC_ENABLE
  // For Pinoccio Scout boards
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000);
#endif

  //////////////AQUÍ VAN LOS MODOS DE PINES DE LOS SENSORES
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SENSOR, INPUT_PULLUP);

  pulseCount = 0;
  flowRate = 0.0;
  flowMilliLitres = 0;
  totalMilliLitres = 0;
  previousMillis = 0;

  attachInterrupt(digitalPinToInterrupt(SENSOR), pulseCounter, FALLING);
  
  ///////////FIN DE AQUÍ VAN LOS MODOS DE PINES DE LOS SENSORES

  //reset OLED display via software
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);

  //initialize OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("LORA SENDER ");
  display.display();
  ////////////////



  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Start job (sending automatically starts OTAA too)
  do_send(&sendjob);
}

void loop() {
  os_runloop_once();
}
