#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <time.h>
#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SDL_Arduino_INA3221.h>

// Device settings
#define DEVICE_NAME "SmartFarmNode_01"

// Pin definitions
#define DHT1_PIN 4     // GPIO4 (D2)
#define DHT2_PIN 5     // GPIO5 (D1)
#define RELAY1_PIN 16  // GPIO16 (D0)
#define RELAY2_PIN 14  // GPIO14 (D5)
#define SDA_PIN 12     // GPIO12 (D6)
#define SCL_PIN 13     // GPIO13 (D7)
#define DHTTYPE DHT11

// INA3221 channels
#define CHANNEL_1 1
#define CHANNEL_2 2
#define CHANNEL_3 3

// Wifi credentials
const char* ssid = "AlphatBits";
const char* password = "secondbrains";

// MQTT Broker settings
const char* mqtt_server = "106fe3e8b38542f1bac5bd11d49751c9.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_username = "alphabits";
const char* mqtt_password = "AlphaBits1";

// MQTT Topics
const char* temp_topic_1 = "smart-farm/node/sensor/innode/temperature";
const char* humid_topic_1 = "smart-farm/node/sensor/innode/humidity";
const char* temp_topic_2 = "smart-farm/node/sensor/outnode/temperature";
const char* humid_topic_2 = "smart-farm/node/sensor/outnode/humidity";
const char* current_topic = "smart-farm/node/sensor/current";
const char* relay1_topic = "smart-farm/node/control/node_1/relay1";
const char* relay2_topic = "smart-farm/node/control/node_1/relay2";

// Time settings
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 7 * 3600;  // GMT+7 for Vietnam
const int daylightOffset_sec = 0;

// Initialize sensors
DHT dht1(DHT1_PIN, DHTTYPE);
DHT dht2(DHT2_PIN, DHTTYPE);
SDL_Arduino_INA3221 ina3221;

// HiveMQ Cloud Certificate
static const char* root_ca PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)EOF";

WiFiClientSecure espClient;
PubSubClient client(espClient);

unsigned long lastMsg = 0;
const int msgInterval = 30000;  // Send data every 30 seconds

void setup_wifi() {
  delay(10);
  Serial.println("Connecting to WiFi");
  WiFi.mode(WIFI_STA);
  WiFi.hostname(DEVICE_NAME);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.printf("Message arrived [%s]: %s\n", topic, message.c_str());

  // Handle relay control
  if (String(topic) == relay1_topic) {
    digitalWrite(RELAY1_PIN, message.toInt() ? HIGH : LOW);
  }
  else if (String(topic) == relay2_topic) {
    digitalWrite(RELAY2_PIN, message.toInt() ? HIGH : LOW);
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect(DEVICE_NAME, mqtt_username, mqtt_password)) {
      Serial.println("connected");
      // Subscribe to relay control topics
      client.subscribe(relay1_topic);
      client.subscribe(relay2_topic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" retrying in 5 seconds");
      delay(5000);
    }
  }
}

String getTime() {
  time_t now;
  struct tm timeinfo;
  char timeStringBuff[50];
  
  time(&now);
  localtime_r(&now, &timeinfo);
  strftime(timeStringBuff, sizeof(timeStringBuff), "%Y-%m-%d %H:%M:%S", &timeinfo);
  return String(timeStringBuff);
}

void publishSensorData(const char* topic, float value, const char* sensorType) {
  StaticJsonDocument<200> doc;
  
  doc["device"] = DEVICE_NAME;
  doc["time"] = getTime();
  doc["type"] = sensorType;
  doc["value"] = value;

  char jsonBuffer[200];
  serializeJson(doc, jsonBuffer);
  
  client.publish(topic, jsonBuffer);
  Serial.printf("Published to %s: %s\n", topic, jsonBuffer);
}

void publishCurrentData() {
  StaticJsonDocument<400> doc;
  JsonArray data = doc.createNestedArray("channels");
  
  for(int channel = 1; channel <= 3; channel++) {
    JsonObject channelObj = data.createNestedObject();
    channelObj["channel"] = channel;
    channelObj["busVoltage"] = ina3221.getBusVoltage_V(channel);
    channelObj["shuntVoltage"] = ina3221.getShuntVoltage_mV(channel);
    channelObj["current"] = ina3221.getCurrent_mA(channel);
  }
  
  doc["device"] = DEVICE_NAME;
  doc["time"] = getTime();
  doc["type"] = "current_sensor";

  char jsonBuffer[400];
  serializeJson(doc, jsonBuffer);
  
  client.publish(current_topic, jsonBuffer);
  Serial.printf("Published current data: %s\n", jsonBuffer);
}

void setup() {
  Serial.begin(115200);
  
  // Initialize GPIO pins
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  digitalWrite(RELAY1_PIN, LOW);
  digitalWrite(RELAY2_PIN, LOW);
  
  // Initialize I2C and sensors
  Wire.begin(SDA_PIN, SCL_PIN);
  dht1.begin();
  dht2.begin();
  ina3221.begin();
  
  setup_wifi();
  
  // Configure SSL/TLS
  espClient.setTrustAnchors(new X509List(root_ca));
  
  // Set clock
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  
  // Configure MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long now = millis();
  if (now - lastMsg > msgInterval) {
    lastMsg = now;
    
    // Read DHT sensors
    float temp1 = dht1.readTemperature();
    float humid1 = dht1.readHumidity();
    float temp2 = dht2.readTemperature();
    float humid2 = dht2.readHumidity();
    
    // Publish DHT data if valid
    if (!isnan(temp1) && !isnan(humid1)) {
      publishSensorData(temp_topic_1, temp1, "temperature");
      publishSensorData(humid_topic_1, humid1, "humidity");
    }
    
    if (!isnan(temp2) && !isnan(humid2)) {
      publishSensorData(temp_topic_2, temp2, "temperature");
      publishSensorData(humid_topic_2, humid2, "humidity");
    }
    
    // Publish current sensor data
    publishCurrentData();
  }
}
