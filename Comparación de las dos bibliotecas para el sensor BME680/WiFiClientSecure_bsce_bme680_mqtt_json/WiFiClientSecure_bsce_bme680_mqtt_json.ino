//IoT-02-09 - Author: Carolina Rodríguez
//Based on https://github.com/jordibinefa/IoT-02/tree/master/codes/IoT-02_altitude_00
//BSD license, all text above must be included in any redistribution

#include <WiFiClientSecure.h>
#include <Wire.h>
#include <SPI.h>
#include <Arduino_JSON.h>
//Software Bosch Sensortec Environmental Cluster (BSEC) for BME680
#include "bsec.h"
#define I2C_SCL 22
#define I2C_SDA 21

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
const char* password = "yourPassword"; // your network password



JSONVar jsonBME;

// Helper functions declarations
void checkIaqSensorStatus(void);
void errLeds(void);

// Create an object of the class Bsec
Bsec iaqSensor;

String output;


/*-----------------------FUNCION DE SETUP BME680 with software BSEC---------------------------------------------------------------------------------------------------------------------------------------------*/
void vsetupBME680_BSEC() {
  Wire.begin(I2C_SDA, I2C_SCL);

  iaqSensor.begin(0x77, Wire);
  checkIaqSensorStatus();

  bsec_virtual_sensor_t sensorList[10] = {
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
  };

  iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
}

void checkIaqSensorStatus(void)
{
  if (iaqSensor.status != BSEC_OK) {
    if (iaqSensor.status < BSEC_OK) {
      output = "BSEC error code : " + String(iaqSensor.status);
      Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      output = "BSEC warning code : " + String(iaqSensor.status);
      Serial.println(output);
    }
  }

  if (iaqSensor.bme680Status != BME680_OK) {
    if (iaqSensor.bme680Status < BME680_OK) {
      output = "BME680 error code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      output = "BME680 warning code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
    }
  }
}
void errLeds(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
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
      client.subscribe(String("/" + String(sMac) + TOPIC_REQUEST_IAQ).c_str());
      client.subscribe(String("/" + String(sMac) + TOPIC_REQUEST_sIAQ).c_str());
      client.subscribe(String("/" + String(sMac) + TOPIC_REQUEST_IAQacc).c_str());
      client.subscribe(String("/" + String(sMac) + TOPIC_REQUEST_CO2).c_str());
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

  if (szTopic == String("/" + String(sMac) + TOPIC_REQUEST_JSON).c_str()) {
    String szJson = sJson();
    Serial.print("JSON: "); Serial.println(szJson);
    client.publish( String("/" + String(sMac) + TOPIC_JSON).c_str(), szJson.c_str());
  }


}

/*-------------------------------FUNCIÓN PARA CREAR JSON-------------------------------------------------------------------------------------------------------------------------------------*/


String sJson() {
  JSONVar json_BME680_BSEC;
  float fTamb, fRH, fP, fGr, fIAQ, fSIAQ, fIAQacc, fCO2;
  static uint32_t nTimes = 0;

  if (iaqSensor.run()) {
    fTamb = iaqSensor.temperature;
    fRH = iaqSensor.humidity;
    fP = iaqSensor.pressure / 100.0;
    fGr = iaqSensor.gasResistance / 1000.0;
    fIAQ = iaqSensor.iaq;
    fSIAQ = iaqSensor.staticIaq;
    fIAQacc = iaqSensor.iaqAccuracy;
    fCO2 = iaqSensor.co2Equivalent;
  }

  json_BME680_BSEC["Tamb"] = fTamb;
  json_BME680_BSEC["RH"] = fRH;
  json_BME680_BSEC["P"] = fP;
  json_BME680_BSEC["Gr"] = fGr;
  json_BME680_BSEC["IAQ"] = fIAQ;
  json_BME680_BSEC["IAQacc"] = fIAQacc;
  json_BME680_BSEC["SIAQ"] = fSIAQ;
  json_BME680_BSEC["CO2"] = fCO2;

  String jsonString = JSON.stringify(json_BME680_BSEC);
  return jsonString;
}

/*--------------------------------PROGRAMA PRINCIPAL-----------------------------------------------------------------------------------------------------------------------------------------*/

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(115200);
  delay(100);
  setup_wifi();
  vSetupMqtt();

  while (!Serial);
  Serial.println(F("BME680 test"));
  vsetupBME680_BSEC();

}

void loop() {


  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}
