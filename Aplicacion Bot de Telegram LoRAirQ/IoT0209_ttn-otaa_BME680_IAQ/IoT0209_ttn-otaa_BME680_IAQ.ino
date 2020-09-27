/****************************************************************************
// IoT-02-09 - Author: Carolina Rodríguez
// Based on https://github.com/G6EJD/BME680-Example/blob/master/ESP32_bme680_CC_demo_03.ino

/****************************************************************************
  This is a library for the BME680 gas, humidity, temperature & pressure sensor
  Designed specifically to work with the Adafruit BME680 Breakout
  ----> http://www.adafruit.com/products/XXXX
  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.
  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!
  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

//---------Arduino+BME68------------------------
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME680 bme; // I2C

float hum_weighting = 0.25; // so hum effect is 25% of the total air quality score
float gas_weighting = 0.75; // so gas effect is 75% of the total air quality score

int   humidity_score, gas_score, iaq_score;
float gas_reference = 2500;
float hum_reference = 40;
int   getgasreference_count = 0;
int   gas_lower_limit = 10000;  // Bad air quality limit
int   gas_upper_limit = 300000; // Good air quality limit
String iaq_text;
float temp, rh, p, air_quality_score;


#define I2C_SDA 21
#define I2C_SCL 22
#define LED_W 19
#define LED_R 23
#define LED_Y 27
#define LED_G 32
#define BT_IO0 0

#include "SSD1306.h"
SSD1306  display(0x3c, I2C_SDA, I2C_SCL);


//---------LORAWAN-----------------------------------------------------------------------------------------------------------------------------
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. 
static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui (u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getDevEui (u1_t* buf) {
  memcpy_P(buf, DEVEUI, 8);
}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getDevKey (u1_t* buf) {
  memcpy_P(buf, APPKEY, 16);
}

static osjob_t sendjob;


int interval_send = 10 * 60 * 1000;  //minutos * segundos * milisegundos = 10 minutos
int interval_read = 10 * 1000; //segundos * milisegundos = 3 segundos
unsigned long previousMillis = 0;
unsigned long previousMillis_read = 0;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// Pin mapping
#define SCK_GPIO     5
#define MISO_GPIO   15
#define MOSI_GPIO   25
#define NSS_GPIO    18 /*16*/
#define RXTX_GPIO   LMIC_UNUSED_PIN
#define RESET_GPIO  14
#define DIO0_GPIO   26
#define DIO1_GPIO   13
#define DIO2_GPIO   12

const lmic_pinmap lmic_pins = {
  .nss = NSS_GPIO,
  .rxtx = RXTX_GPIO,
  .rst = RESET_GPIO,
  .dio = {DIO0_GPIO, DIO1_GPIO, DIO2_GPIO},
};


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

      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
        for (int i = 0; i < LMIC.dataLen; i++) {
          if (LMIC.frame[LMIC.dataBeg + i] < 0x10) {
            Serial.print(F("0"));
          }
          Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
          Serial.print(" ");
        }
        Serial.println();
      }

      // Schedule next transmission
      // os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
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
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}

void do_send(osjob_t* j) {
  int T = (int)(temp * 10.0);
  int P = (int)(p * 10.0);
  int RH = (int)(rh * 10.0);
  int GAS = (int)(gas_reference / 1000.0);
  int IAQ = (int)((100 - air_quality_score) * 5);


  byte payload[10];
  payload[0] = highByte(T);
  payload[1] = lowByte(T);
  payload[2] = highByte(P);
  payload[3] = lowByte(P);
  payload[4] = highByte(RH);
  payload[5] = lowByte(RH);
  payload[6] = highByte(GAS);
  payload[7] = lowByte(GAS);
  payload[8] = highByte(IAQ);
  payload[9] = lowByte(IAQ);

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, payload, sizeof(payload), 0);
    Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.

}

//----------------------------------setup arduino-------------------------------------
void vSetupIO() {
  pinMode(LED_G, OUTPUT);
  pinMode(LED_Y, OUTPUT);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_W, OUTPUT);
  pinMode(BT_IO0, INPUT);
}

boolean bPressedButton(int nWhichOne) {
  if (digitalRead(nWhichOne))
    return false;
  return true;
}

//-----------------------------------Screen OLED---------------------------------------
void vSetupScreen() {
  display.init();
  display.flipScreenVertically();
}
void vScreenDemo(int iaqscore, String iaqtext, int nCmpt) {
  iaqscore = (100 - iaqscore) * 5;
  int nAra = millis() / 1000;
  static int nDarrerAra = nAra, nTx100, nPx100, nRHx100, nGr, nAx100;
  display.clear();

  // create more fonts at http://oleddisplay.squix.ch/
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  if (nCmpt % 2) {
    display.drawString(0, 0, "T: " + String(temp, 2) + " *C");
    display.drawString(0, 10, "RH: " + String(rh, 1) + " %");
    display.drawString(0, 20, "P: " + String(p, 1) + " hPa");
    display.drawString(0, 30, "G: " + String(gas_reference / 1000.0F) + "KOhms");
    display.drawString(0, 40, "IQA: " + String(iaqscore) + " " + String(iaqtext));
  } else {
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, "Air Quality Index: ");
    display.setFont(ArialMT_Plain_24);
    display.drawString(36, 18, String(iaqscore));
    display.drawString(12, 38, String(iaq_text));
  }
  display.display();
}
//---------------------------setup BME680-----------------------------------------------
void vsetupBME680() {
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }
  else Serial.println("Found a sensor");

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320°C for 150 ms

  GetGasReference();
}


void GetGasReference() {
  // Now run the sensor for a burn-in period, then use combination of relative humidity and gas resistance to estimate indoor air quality as a percentage.
  //Serial.println("Getting a new gas reference value");
  int readings = 10;
  for (int i = 1; i <= readings; i++) { // read gas for 10 x 0.150mS = 1.5secs
    gas_reference += bme.readGas();
  }
  gas_reference = gas_reference / readings;
  //Serial.println("Gas Reference = "+String(gas_reference,3));
}

String CalculateIAQ(int score) {
  String IAQ_text;
  score = (100 - score) * 5;
  if      (score >= 151) {
    IAQ_text = "Unhealthy";
    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_Y, LOW);
    digitalWrite(LED_G, LOW);
  }
  else if (score >=  51 && score <= 150 ) {
    IAQ_text = "Moderate";
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_Y, HIGH);
    digitalWrite(LED_G, LOW);
  }
  else if (score >=  00 && score <=  50 ) {
    IAQ_text = "Good";
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_Y, LOW);
    digitalWrite(LED_G, HIGH);
  }
  return IAQ_text;
}

int GetHumidityScore() {  //Calculate humidity contribution to IAQ index
  float current_humidity = bme.readHumidity();
  if (current_humidity >= 38 && current_humidity <= 42) // Humidity +/-5% around optimum
    humidity_score = 0.25 * 100;
  else
  { // Humidity is sub-optimal
    if (current_humidity < 38)
      humidity_score = 0.25 / hum_reference * current_humidity * 100;
    else
    {
      humidity_score = ((-0.25 / (100 - hum_reference) * current_humidity) + 0.416666) * 100;
    }
  }
  return humidity_score;
}

int GetGasScore() {
  //Mejora de la sensibilidad del AIQ calculado//Carol Rodriguez
  if (gas_reference > gas_upper_limit) {
    gas_upper_limit = gas_reference;
  }

  //Calculate gas contribution to IAQ index
  gas_score = (0.75 / (gas_upper_limit - gas_lower_limit) * gas_reference - (gas_lower_limit * (0.75 / (gas_upper_limit - gas_lower_limit)))) * 100.00;
  if (gas_score > 75) gas_score = 75; // Sometimes gas readings can go outside of expected scale maximum
  if (gas_score <  0) gas_score = 0;  // Sometimes gas readings can go outside of expected scale minimum
  return gas_score;
}

//--------PROGRAMA PRINCIPAL-----------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println(F("Starting"));

  // LMIC init
  SPI.begin(SCK_GPIO, MISO_GPIO, MOSI_GPIO, NSS_GPIO);
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  vSetupIO();
  vSetupScreen();
  vsetupBME680();

  temp = bme.readTemperature();
  p = (bme.readPressure() / 100.0F);
  rh = bme.readHumidity();

  humidity_score = GetHumidityScore();
  gas_score      = GetGasScore();

  static int nCmptIO0 = 0;
  air_quality_score = humidity_score + gas_score;
  if ((getgasreference_count++) % 5 == 0) GetGasReference();
  iaq_text = CalculateIAQ(air_quality_score);
  vScreenDemo(air_quality_score, iaq_text, nCmptIO0);

  do_send(&sendjob);

}

void loop() {
  static int nCmptIO0 = 0;
  static boolean bIO0wasPressed = false;
  boolean bIO0currentState = bPressedButton(BT_IO0);

  if (bIO0wasPressed != bIO0currentState) {
    delay(2);
    if (bIO0currentState) {
      nCmptIO0++;
    }
    bIO0wasPressed = bIO0currentState;
  }


  os_runloop_once();

  unsigned long currentMillis = millis();
  if ((unsigned long)(currentMillis - previousMillis) >= interval_send) {
    do_send(&sendjob);
    previousMillis = currentMillis;
  }

  unsigned long currentMillis_read = millis();
  if ((unsigned long)(currentMillis_read - previousMillis_read) >= interval_read) {

    previousMillis_read = currentMillis_read;
    temp = bme.readTemperature();
    p = (bme.readPressure() / 100.0F);
    rh = bme.readHumidity();

    humidity_score = GetHumidityScore();
    gas_score      = GetGasScore();
    air_quality_score = humidity_score + gas_score;
    if ((getgasreference_count++) % 5 == 0) GetGasReference();
    iaq_text = CalculateIAQ(air_quality_score);
    vScreenDemo(air_quality_score, iaq_text, nCmptIO0);
  }

}
