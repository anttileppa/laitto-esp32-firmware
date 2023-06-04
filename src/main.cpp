#include <sstream>
#include <WiFiClient.h>
#include <MQTTClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ETH.h>
#include "secrets.h"

// Constants
#define NETWORK_CONNECTION_TIMEOUT_MS 15000
#define MQTT_CONNECT_TIMEOUT 10000

// Temperature sensors
OneWire oneWire(TEMPERATURE_SENSOR_PIN);
DallasTemperature sensors(&oneWire);

// Network
WiFiClient net = WiFiClient();
bool ethConnected = false;

// MQTT
MQTTClient client = MQTTClient(4096);

// Variables
int temperatureSensorCount = 0;
DeviceAddress temperatureDeviceAddress;
unsigned long lastMqttConnection = 0;
String deviceId = "";

/**
 * Connect ESP to wifi
 */
void connectWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.println("Connecting to Wi-Fi");

  while (WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }

  Serial.println("Connected to Wi-Fi with device id " + WiFi.macAddress());
}

/**
 * Connect to MQTT 
*/
void connectToMQTT() {
  Serial.print("Setting MQTT settings (");
  Serial.print("user: ");
  Serial.print(MQTT_USER);
  Serial.print(", pass: ");
  Serial.print(MQTT_PASSWORD);
  Serial.print(", server: ");
  Serial.print(MQTT_SERVER);
  Serial.println(")");

  client.begin(MQTT_SERVER, MQTT_PORT, net);

  client.setOptions(10, true, 5000);

  char clientId[deviceId.length() + 1];
  deviceId.toCharArray(clientId, deviceId.length() + 1);

  Serial.println("Connecting to MQTT endpoint...");
  long connectionStarted = millis();
  while (!client.connect(clientId, MQTT_USER, MQTT_PASSWORD)) {
    Serial.println(client.lastError());
    Serial.print(".");
  }

  Serial.println("MQTT connected!");
}

/**
 * Convert array of bytes to hex string
 * 
 * @param arr array of bytes
 * @param size size of the array
 * @return hex string
 */
std::string arrayToHexString(uint8_t* arr, size_t size) {
    std::ostringstream result;
    
    for(size_t i = 0; i < size; ++i) {
        result << std::hex << static_cast<int>(arr[i]);
    }

    return result.str();
}

/**
 * Send temperature data to MQTT
 * 
 * @param deviceAddress device address for the temperature sensor
 */
void sendTemperatureData(DeviceAddress deviceAddress) {
  float tempC = sensors.getTempC(deviceAddress);
  StaticJsonDocument<200> doc;
  doc["temperature"] = tempC;
  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer);
  Serial.println(jsonBuffer);

  std::ostringstream channel;
  channel << "devices/" << arrayToHexString(deviceAddress, 8) << "/status/temperature";

  client.publish(channel.str().c_str(), jsonBuffer);
};

/**
 * Setup function
 */
void setup() {
  // Init serial
  Serial.begin(9600);

  // Init temperature sensors
  sensors.begin();
  temperatureSensorCount = sensors.getDeviceCount();

  // Init network
  deviceId = WiFi.macAddress();
  connectWifi();

  // Init MQTT
  connectToMQTT();
}

/**
 * Main loop
 */
void loop() {
  sensors.requestTemperatures();

  for (int temperatureSensorIndex = 0; temperatureSensorIndex < temperatureSensorCount; temperatureSensorIndex++) {
    if (sensors.getAddress(temperatureDeviceAddress, temperatureSensorIndex)) {
        sendTemperatureData(temperatureDeviceAddress);
    }
  }

  client.loop();

  delay(1000);

  if (WiFi.status() != WL_CONNECTED){
    Serial.println("Lost connection to Wi-Fi, restarting ESP..."); 
    esp_restart();
  }

  if (!client.connected()) {
    Serial.println("Lost connection to MQTT, restarting ESP...");
    esp_restart();
  }
}
