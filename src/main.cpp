#include <sstream>
#include <WiFiClient.h>
#include <MQTTClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ETH.h>
#include "secrets.h"

/**
 * Pin mapping for relays
 */
static const int relayToPinMapping[16] = {
  00, 00, 00, 00, // Relay 1 to 4
  00, 00, 00, 00, // Relay 5 to 8
  00, 00, 00, 00, // Relay 9 to 12
  00, 00, 10, 11  // Relay 13 to 16
};

/**
 * Relay states
 */
static bool relayOn[16] = {
  false, false, false, false, // Relay 1 to 4
  false, false, false, false, // Relay 5 to 8
  false, false, false, false, // Relay 9 to 12
  false, false, false, false  // Relay 13 to 16
};

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
 * Sets relay active or inactive
 * 
 * @param relayIndex index of the relay
 * @param active true if relay should be active, false otherwise
 */
void setRelayActive(int relayIndex, bool active, bool force = false) {
  if (!force && relayOn[relayIndex] == active) {
    Serial.println("Relay " + String(relayIndex) + " is already " + String(active ? "active" : "deactive"));
    return;
  }
    
  int pin = relayToPinMapping[relayIndex];
  if (pin == 0) {
    Serial.println("Relay index " + String(relayIndex) + " is not mapped to any pin");
    return;
  } else {
    Serial.println("Setting relay " + String(relayIndex) + " to " + String(active ? "active" : "deactive"));
    digitalWrite(pin, active ? LOW : HIGH);
    relayOn[relayIndex] = active;
  }
};

/**
 * MQTT message received callback
 * 
 * @param topic topic of the message
 * @param payload payload of the message
 */
void messageReceived(String &topic, String &payload) {
  if (topic.startsWith("devices/") && topic.endsWith("/changeStatus")) {
    // It's a device status change message
    String deviceId = topic.substring(topic.indexOf('/') + 1, topic.lastIndexOf('/')); 
    int relayIndex = deviceId.indexOf("-relay-");
    
    if (relayIndex > -1) {
      // ... for a relay
      String relayIndexString = deviceId.substring(relayIndex + 7);
      int relayIndex = relayIndexString.toInt();

      StaticJsonDocument<200> doc;
      DeserializationError error = deserializeJson(doc, payload);
      
      if (error) {
        Serial.println("Failed to parse JSON");
        return;
      }

      setRelayActive(relayIndex, doc["status"] > 0);
    }
  }
}

/**
 * Publishes online message to mqtt broker
 */
void publishOnlineMqttMessage() {
  StaticJsonDocument<200> doc;
  doc["status"] = "online";
  doc["version"] = VERSION_NAME;
  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer);
  client.publish("devices/" + deviceId + "/status", jsonBuffer);
}

/**
 * Returns short device id (without colons), e.g. 1234567890
 * 
 * @return short device id
 */
String getShortDeviceId() {
  String output = "";

  for (int i = 0; i < deviceId.length(); i++) {
    if (deviceId[i] != ':') {
      output += deviceId[i];
    }
  }

  return output;
}

/**
 * Returns relay id. Id is in format <device_id>-relay-<index> (e.g. 1234567890-relay-1).
 * 
 * @param index index of the relay
 * @return relay id 
 */
String getRelayId(int index) {
  return getShortDeviceId() + "-relay-" + String(index);
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

  for (int i = 0; i < 16; i++) {
    client.subscribe("devices/" + getRelayId(i) + "/changeStatus");
  };

  client.onMessage(messageReceived);
  publishOnlineMqttMessage();

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

  std::string channel = "devices/" + arrayToHexString(deviceAddress, 8) + "/status/temperature";
  client.publish(channel.c_str(), jsonBuffer);

  Serial.print(channel.c_str());
  Serial.print(": ");
  Serial.println(jsonBuffer);
};

/**
 * Initialize relays by setting pin mode to output 
 */
void initialiseRelays() {
  for (int i = 0; i < 16; i++) {
    int pin = relayToPinMapping[i];
    if (pin != 0) {
      pinMode(pin, OUTPUT);
      setRelayActive(i, false, true);
    }
  }
}

/**
 * Setup function
 */
void setup() {
  // Init serial
  Serial.begin(9600);

  // Initialize relays
  initialiseRelays();
  
  // Init temperature sensors
  sensors.begin();
  temperatureSensorCount = sensors.getDeviceCount();

  // Init network
  deviceId = WiFi.macAddress();

  // Connect to wifi
  connectWifi();

  // Init MQTT
  connectToMQTT();

  Serial.println("Setup done!");
  Serial.println("Device id: " + deviceId);
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
