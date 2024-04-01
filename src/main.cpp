#include <sstream>
#include <WiFiClient.h>
#include <MQTTClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ETH.h>
#include "ota-update.h"
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
#define TEMPERATURE_SENSOR_PIN 13
#define OTA_CHECK_INTERVAL_MS 60000
#define ONLINE_MESSAGE_INTERVAL_MS 10000
#define TEMPERATURE_PUBLISH_INTERVAL_MS 5000
#define LOG_TO_MQTT true
#define SSR_RETENTION_TIME_MS 10000

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
unsigned long lastOtaCheck = 0;
unsigned long ssrState = 0;
bool ssrActive = false;
unsigned long ssrOffTime = 0;
unsigned long ssrStateChanged = 0;
unsigned long lastOnlineMessage = 0;
unsigned long lastTemperaturePublish = 0;

/**
 * Write log message
 * 
 * @param message message to write
 */
void writeLog(const String &message) {
  if (LOG_TO_MQTT) {
    client.publish("devices/" + deviceId + "/log", message);
  } else {
    Serial.println(message);
  }
}

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
    writeLog("Relay " + String(relayIndex) + " is already " + String(active ? "active" : "deactive"));
    return;
  }
  
  int pin = relayToPinMapping[relayIndex];
  if (pin == 0) {
    writeLog("Relay index " + String(relayIndex) + " is not mapped to any pin");
    return;
  } else {
    writeLog("Setting relay " + String(relayIndex) + " to " + String(active ? "active" : "deactive"));
    digitalWrite(pin, active ? LOW : HIGH);
    relayOn[relayIndex] = active;
  }
};

/**
 * Parses change status message status
 * 
 * @param payload payload of the message
 * @return status
 */
int parseChangeStatusMessageStatus(String &payload) {
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, payload);
  
  if (error) {
    writeLog("Failed to parse JSON");
    return -1;
  }

  return doc["status"]; 
}

/**
 * Calculate SSR off time based on SSR state. This is used to calculate how long the SSR should be off
 * 
 * Example: if SSR_RETENTION_TIME_MS is 10000 and SSR state is 50, the SSR should be off for 5000 ms
 * Example: if SSR_RETENTION_TIME_MS is 10000 and SSR state is 90, the SSR should be off for 1000 ms
*/
int calculateSsrOffTime() {
  return SSR_RETENTION_TIME_MS * (100 - ssrState) / 100;
}

/**
 * Update SSR state and calculate new interval
 * 
 * @param newState new state of the SSR
 */
void updateSsrState(int newState) {
  ssrState = newState;
  ssrOffTime = calculateSsrOffTime();
  writeLog("SSR state: " + String(ssrState) + ", ssr off time: " + String(ssrOffTime));
}

/**
 * MQTT message received callback
 * 
 * @param topic topic of the message
 * @param payload payload of the message
 */
void messageReceived(String &topic, String &payload) {
  writeLog("Received message");

  if (topic.startsWith("devices/") && topic.endsWith("/changeStatus")) {
    // It's a device status change message

    writeLog("Received message: " + topic + " - " + payload);
    
    String deviceId = topic.substring(topic.indexOf('/') + 1, topic.lastIndexOf('/')); 
    int status = parseChangeStatusMessageStatus(payload);
    if (status == -1) {
      return;
    }
    
    int relayIndex = deviceId.indexOf("-relay-");
    
    if (relayIndex > -1) {
      // ... for a relay

      String relayIndexString = deviceId.substring(relayIndex + 7);
      int relayIndex = relayIndexString.toInt();
      setRelayActive(relayIndex, status > 0);
    } else if (deviceId.endsWith("-ssr")) {
      // ... for SSR
      updateSsrState(status);
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
 * Returns SSR id. Id is in format <device_id>-ssr (e.g. 1234567890-ssr).
 * 
 * @return SSR id
 */
String getSsrId() {
  return getShortDeviceId() + "-ssr";
}

/**
 * Subscribe to MQTT topic
 * 
 * @param topic topic to subscribe to
 */
void mqttSubscribe(const String &topic) {
  writeLog("MQTT subscribing to " + topic);
  client.subscribe(topic);
};

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
    mqttSubscribe("devices/" + getRelayId(i) + "/changeStatus");
  };

  mqttSubscribe("devices/" + getSsrId() + "/changeStatus");

  client.onMessage(messageReceived);
  publishOnlineMqttMessage();

  writeLog("MQTT connected!");
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
void publishTemperatureChange(DeviceAddress deviceAddress) {
  float tempC = sensors.getTempC(deviceAddress);
  StaticJsonDocument<200> doc;
  doc["temperature"] = tempC;
  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer);

  std::string channel = "devices/" + arrayToHexString(deviceAddress, 8) + "/status/temperature";
  client.publish(channel.c_str(), jsonBuffer);
};

/**
 * Publishes device online message
 * 
 * @param deviceId device id
 * @param status status of the device
 */
void publishDeviceOnline(const String &deviceId, const int status) {
  StaticJsonDocument<200> doc;
  doc["status"] = status;
  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer);
  client.publish("devices/" + deviceId + "/online", jsonBuffer);
}

/**
 * Publishes device onlines
 */
void publishDeviceOnlines() {
  writeLog("Publishing device onlines");
  
  for (int i = 0; i < 16; i++) {
    publishDeviceOnline(getRelayId(i), relayOn[i]);
  }

  publishDeviceOnline(getSsrId(), ssrState);
}

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
 * Initialize SSR by setting pin mode to output
 */
void initSsr() {
  pinMode(SSR_PIN, OUTPUT);
}

/**
 * Setup function
 */
void setup() {
  // Init serial
  Serial.begin(9600);

  // Initialize relays
  initialiseRelays();

  // Init SSR
  initSsr();

  // Init temperature sensors
  sensors.begin();
  temperatureSensorCount = sensors.getDeviceCount();

  // Init network
  deviceId = WiFi.macAddress();

  // Connect to wifi
  connectWifi();

  // Init MQTT
  connectToMQTT();

  // Publish online message
  publishDeviceOnlines();

  Serial.println("Setup done!");
  Serial.println("Version: " + getCurrentVersionName());
  Serial.println("Device id: " + deviceId);
}

/**
 * Set SSR active or inactive
 * 
 * @param active true if SSR should be active, false otherwise
 */
void setSsrActive(bool active) {
  ssrActive = active;
  ssrStateChanged = millis();
  writeLog("Setting SSR to " + String(ssrActive ? "active" : "inactive"));
  digitalWrite(SSR_PIN, ssrActive ? HIGH : LOW);
}

/**
 * Main loop
 */
void loop() {
  if (millis() - lastTemperaturePublish > TEMPERATURE_PUBLISH_INTERVAL_MS) {
    Serial.println("Requesting temperatures");

    lastTemperaturePublish = millis();
    sensors.requestTemperatures();
    for (int temperatureSensorIndex = 0; temperatureSensorIndex < temperatureSensorCount; temperatureSensorIndex++) {
      if (sensors.getAddress(temperatureDeviceAddress, temperatureSensorIndex)) {
          publishTemperatureChange(temperatureDeviceAddress);
      }
    }
  }

  client.loop();

  if (WiFi.status() != WL_CONNECTED){
    Serial.println("Lost connection to Wi-Fi, restarting ESP..."); 
    esp_restart();
  }

  if (!client.connected()) {
    Serial.println("Lost connection to MQTT, restarting ESP...");
    esp_restart();
  }

  if (ssrState > 0 && millis() - lastOtaCheck > OTA_CHECK_INTERVAL_MS) {
    lastOtaCheck = millis();
    checkFirmwareUpdates();
  }

  if (ssrState == 0 && ssrActive) {
    setSsrActive(false);
  } else if (ssrState == 100 && !ssrActive) {
    setSsrActive(true);
  } else if (ssrState > 0 && ssrState < 100) {
    if (ssrActive && (millis() - ssrStateChanged) > (SSR_RETENTION_TIME_MS - ssrOffTime)) {
      setSsrActive(false);
    } else if (!ssrActive && millis() - ssrStateChanged > ssrOffTime) {
      setSsrActive(true);
    }
  }

  if (millis() - lastOnlineMessage > ONLINE_MESSAGE_INTERVAL_MS) {
    lastOnlineMessage = millis();
    publishDeviceOnlines();
  }
}
