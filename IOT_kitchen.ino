#include <WiFi.h>
#include <PubSubClient.h>
#include "DHT.h"
#include <Adafruit_NeoPixel.h>

// ---------------------- USER CONFIG ----------------------

// WiFi credentials
const char* ssid     = "Test";
const char* password = "123456789";

// MQTT broker (public test broker)
const char* mqttServer = "test.mosquitto.org";
const int   mqttPort   = 1883;

// MQTT topics
const char* topicTemp      = "kitchen/dht/temperature";
const char* topicHum       = "kitchen/dht/humidity";
const char* topicAlarmSub  = "kitchen/alarm";   // ESP32 listens here

// DHT sensor config
#define DHTPIN 4        // <- CHANGE if your DHT11 is on another GPIO
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// NeoPixel config
#define NEOPIXEL_PIN 5  // <- CHANGE to the pin where your LED strip/data line is connected
#define NUM_PIXELS   8  // <- CHANGE to how many NeoPixel LEDs you have
Adafruit_NeoPixel pixels(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// LED alarm states
// We'll hold current desired alarm mode here
// "OFF"  -> safe (green solid)
// "ON"   -> alarm (red blink)
String alarmState = "OFF";

// Blink timing
unsigned long lastBlinkToggle = 0;
bool blinkOn = false;

// Publish timing
unsigned long lastPublish = 0;
const unsigned long publishInterval = 5000; // ms

// ---------------------------------------------------------

WiFiClient espClient;
PubSubClient client(espClient);

// ---------------------- LED HELPERS ----------------------

void showSolid(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < NUM_PIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(r, g, b));
  }
  pixels.show();
}

void showBlinkRed() {
  unsigned long now = millis();
  if (now - lastBlinkToggle > 300) { // blink every 300 ms
    lastBlinkToggle = now;
    blinkOn = !blinkOn;
    if (blinkOn) {
      showSolid(255, 0, 0);  // RED
    } else {
      showSolid(0, 0, 0);    // OFF
    }
  }
}

void showStateSafe() {
  // Green solid = safe/idle
  showSolid(0, 150, 0);
}

void showStateWarning() {
  // Yellow solid = warning (not used automatically in this code,
  // but you can call this if you want >40C but <50C)
  showSolid(200, 120, 0);
}

void showStateAlarm() {
  // Blinking red (handled in loop)
  showBlinkRed();
}

// ---------------------- MQTT CALLBACK ----------------------

void callback(char* topic, byte* payload, unsigned int length) {
  // This is called whenever we receive a subscribed MQTT message
  // We're only subscribing to "kitchen/alarm"
  if (strcmp(topic, topicAlarmSub) == 0) {
    // Build string from payload bytes
    String cmd;
    for (unsigned int i = 0; i < length; i++) {
      cmd += (char)payload[i];
    }

    cmd.trim();
    Serial.print("[MQTT-IN] kitchen/alarm = ");
    Serial.println(cmd);

    if (cmd == "ON") {
      alarmState = "ON";
    } else {
      alarmState = "OFF";
    }
  }
}

// ---------------------- MQTT RECONNECT ----------------------

void reconnect() {
  // Loop until we're reconnected to MQTT
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection... ");
    // Attempt to connect (client id must be unique-ish, so add a random number)
    String clientId = "ESP32_Kitchen_Node_";
    clientId += String(random(0xffff), HEX);

    if (client.connect(clientId.c_str())) {
      Serial.println("connected ✅");
      // Subscribe to alarm control topic from Node-RED dashboard
      client.subscribe(topicAlarmSub);
      Serial.println("Subscribed to kitchen/alarm");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println("  retrying in 2s...");
      delay(2000);
    }
  }
}

// ---------------------- WIFI CONNECT ----------------------

void connectWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
  }

  Serial.println("");
  Serial.print("WiFi connected. IP: ");
  Serial.println(WiFi.localIP());
}

// ---------------------- SETUP ----------------------

void setup() {
  Serial.begin(115200);

  pixels.begin();
  pixels.clear();
  pixels.show();
  showStateSafe(); // default safe state on boot

  dht.begin();

  connectWiFi();

  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);

  reconnect(); // connect MQTT and subscribe
}

// ---------------------- LOOP ----------------------

void loop() {
  // Keep WiFi + MQTT alive
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // 1. Handle LED animation state
  if (alarmState == "ON") {
    // Blink red
    showStateAlarm();
  } else {
    // Solid green
    showStateSafe();
  }

  // 2. Publish sensor data every 5 seconds
  unsigned long now = millis();
  if (now - lastPublish > publishInterval) {
    lastPublish = now;

    float t = dht.readTemperature(); // Celsius
    float h = dht.readHumidity();    // Percent

    if (isnan(t) || isnan(h)) {
      Serial.println("[WARN] DHT read failed ❌");
    } else {
      // Print to Serial for debugging
      Serial.print("[PUB] Temp=");
      Serial.print(t, 2);
      Serial.print(" °C | Hum=");
      Serial.print(h, 2);
      Serial.println(" %");

      // Convert to char buffers for MQTT publish
      char tempStr[16];
      char humStr[16];
      dtostrf(t, 4, 2, tempStr); // -> "27.45"
      dtostrf(h, 4, 2, humStr);  // -> "61.22"

      // Send to Node-RED
      client.publish(topicTemp, tempStr);
      client.publish(topicHum, humStr);
    }
  }

  // tiny delay for stability
  delay(10);
}