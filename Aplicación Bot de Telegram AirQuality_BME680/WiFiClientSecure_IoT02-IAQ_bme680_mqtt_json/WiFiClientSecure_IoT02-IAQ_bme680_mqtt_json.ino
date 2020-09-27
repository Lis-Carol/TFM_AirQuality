/****************************************************************************
//Author: Carolina Rodriguez
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


#include <Arduino.h>
#include <WiFiClientSecure.h>
#include <Wire.h>
#include <SPI.h>
#include <Arduino_JSON.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
Adafruit_BME680 bme;

#include "IoT-02_mqttCredentials.h"
#include "IoT-02_mqttTopics.h"
#include <PubSubClient.h>
#include <ESPmDNS.h>
WiFiClientSecure espClient;
PubSubClient client(espClient);
#define MAC_SIZE 15
char sMac[MAC_SIZE];
long lastMsg = 0;
char msg[50];
int value = 0;
int touch_value = 0;

const char* ssid     = "user";     // your network SSID (name of wifi network)
const char* password = "yourpassword"; // your network password



#define SEALEVELPRESSURE_HPA (1013.25)

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
    display.drawString(0, 30, "G: " + String(gas_reference / 1000.0F) + " KOhms");
    display.drawString(0, 40, "IQA: " + String(iaqscore) + " " + String(iaqtext));
    display.drawString(0, 50, "gas max: " + String(gas_upper_limit / 1000.0F) + + " KOhms");
  } else {
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, "Air Quality Index: ");
    display.setFont(ArialMT_Plain_24);
    display.drawString(36, 18, String(iaqscore));
    display.drawString(12, 38, String(iaq_text));
  }
  display.display();
}


/*-----------------------FUNCION DE SETUP BME680---------------------------------------------------------------------------------------------------------------------------------------------*/
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
  Serial.print("IAQ Score = " + String(score) + ", ");
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

/*---------------------FUNCIONES WIFI Y MAC--------------------------------------------------------------------------------------------------------------------------------------------------*/
#define N_WIFIS 9
#define MAX_STRING_SIZE 15

struct stWifiList {
  const char* ssid;
  const char* password;
};

/*struct*/ stWifiList stWiFi[N_WIFIS] = {
  {"vodafone7F60" , "PN92VDQC8NUJJT"},
  {"AndroidAPCarol" , "licaroci"},
};

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  szGetMac().toCharArray(sMac, MAC_SIZE);
  Serial.println(sMac);
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

String szGetMac()
{
  byte mac[6];
  String szMAC = "";
  char szMac[3];

  WiFi.macAddress(mac);
  //for (int i = 5; i >= 0; i--)
  for (int i = 0; i < 6; i++)
  {
    if (mac[i] > 0x0F)
      sprintf(szMac, "%2X", mac[i]);
    else
      sprintf(szMac, "0%X", mac[i]);
    szMAC += szMac;
  }
  return szMAC;
}

/*-----------------------FUNCIONES DE MQTT CONECCTION----------------------------------------------------------------------------------------------------------------------------------------*/

void vSetupMqtt() {
  /* set SSL/TLS certificate */
  espClient.setCACert(ca_cert);
  /* configure the MQTT server with IPaddress and port */
  client.setServer(mqtt_server, mqtt_port);
  /* this receivedCallback function will be invoked
    when client received subscribed topic */
  client.setCallback(callback);
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "BreathVOC_" + String(sMac);
    // Attempt to connect
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
      Serial.println("connected");
      client.subscribe(String("/" + String(sMac) + TOPIC_REQUEST_T).c_str());
      client.subscribe(String("/" + String(sMac) + TOPIC_REQUEST_RH).c_str());
      client.subscribe(String("/" + String(sMac) + TOPIC_REQUEST_P).c_str());
      client.subscribe(String("/" + String(sMac) + TOPIC_REQUEST_G).c_str());
      client.subscribe(String("/" + String(sMac) + TOPIC_REQUEST_AIQ).c_str());
      client.subscribe(String("/" + String(sMac) + TOPIC_REQUEST_JSON).c_str());
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {

  String szTopic = String(topic), szPayload = "";
  Serial.println();

  if (szTopic == String("/" + String(sMac) + TOPIC_REQUEST_T).c_str()) {
    String szTemp = String(temp);
    Serial.print("Temperature: "); Serial.println(szTemp);
    client.publish( String("/" + String(sMac) + TOPIC_T).c_str(), szTemp.c_str());
  }
  if (szTopic == String("/" + String(sMac) + TOPIC_REQUEST_RH).c_str()) {
    String szRH = String(rh);
    Serial.print("Humidity: "); Serial.println(szRH);
    client.publish( String("/" + String(sMac) + TOPIC_RH).c_str(), szRH.c_str());
  }
  if (szTopic == String("/" + String(sMac) + TOPIC_REQUEST_P).c_str()) {
    String szP = String(p);
    Serial.print("Pressure: "); Serial.println(szP);
    client.publish( String("/" + String(sMac) + TOPIC_P).c_str(), szP.c_str());
  }
  if (szTopic == String("/" + String(sMac) + TOPIC_REQUEST_G).c_str()) {
    String szGas = String(gas_reference / 1000.0F);
    Serial.print("Gas: "); Serial.println(szGas);
    client.publish( String("/" + String(sMac) + TOPIC_G).c_str(), szGas.c_str());
  }
  if (szTopic == String("/" + String(sMac) + TOPIC_REQUEST_AIQ).c_str()) {
    String szAIQ = String((100 - air_quality_score) * 5);
    Serial.print("AIQ: "); Serial.println(szAIQ);
    client.publish( String("/" + String(sMac) + TOPIC_AIQ).c_str(), szAIQ.c_str());
  }

  if (szTopic == String("/" + String(sMac) + TOPIC_REQUEST_JSON).c_str()) {
    String szJson = sJson();
    Serial.print("JSON: "); Serial.println(szJson);
    client.publish( String("/" + String(sMac) + TOPIC_JSON).c_str(), szJson.c_str());
  }
}

/*-------------------------------FUNCIÓN PARA CREAR JSON-------------------------------------------------------------------------------------------------------------------------------------*/

String sJson() {
  JSONVar json_AIQ;
  float fT, fRH, fP, fGr, fAIQ;

  fT = temp;
  fRH = rh;
  fP = p;
  fGr = (gas_reference / 1000.0F);
  fAIQ = (100 - air_quality_score) * 5;

  json_AIQ["T"] = fT;
  json_AIQ["RH"] = fRH;
  json_AIQ["P"] = fP;
  json_AIQ["Gr"] = fGr;
  json_AIQ["AIQ"] = fAIQ;

  String jsonString = JSON.stringify(json_AIQ);
  return jsonString;

}

/*--------------------------------PROGRAMA PRINCIPAL-----------------------------------------------------------------------------------------------------------------------------------------*/

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(115200);
  Serial.println(__FILE__);
  vsetupBME680();
  setup_wifi();
  vSetupMqtt();
  vSetupIO();
  vSetupScreen();


}

void loop() {
  if (! bme.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

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

  temp = bme.readTemperature();
  p = (bme.readPressure() / 100.0F);
  rh = bme.readHumidity();

  Serial.println("Sensor Readings:");
  Serial.println("  Temperature = " + String(temp, 2)     + "°C");
  Serial.println("     Pressure = " + String(p) + " hPa");
  Serial.println("     Humidity = " + String(rh, 1)        + "%");
  Serial.println("          Gas = " + String(gas_reference)               + " ohms\n");
  Serial.print("Qualitative Air Quality Index ");

  humidity_score = GetHumidityScore();
  gas_score      = GetGasScore();

  //Combine results for the final IAQ index value (0-100% where 100% is good quality air)
  air_quality_score = humidity_score + gas_score;
  Serial.println(" comprised of " + String(humidity_score) + "% Humidity and " + String(gas_score) + "% Gas");
  if ((getgasreference_count++) % 5 == 0) GetGasReference();
  iaq_text = CalculateIAQ(air_quality_score);
  Serial.println(iaq_text);
  Serial.println("--------------------------------------------------------------");
  vScreenDemo(air_quality_score, iaq_text, nCmptIO0);
}
