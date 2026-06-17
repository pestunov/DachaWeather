#include <Arduino.h>
#include <Wire.h>
#include <DHT.h>
#include <WiFiManager.h>
#include <PubSubClient.h>

// ============== КОНФИГУРАЦИЯ ==============
#define DHTPIN          4
#define DHTTYPE         DHT22
#define I2C_SDA         21
#define I2C_SCL         22
#define BMP180_ADDR     0x77
#define JSON_BUFFER_SIZE 128
#define READ_INTERVAL   10000      // 10 секунд
#define MQTT_BROKER     "192.168.1.107"  // IP БРОКЕРА!
#define MQTT_PORT       1883

// ============== ОБЪЕКТЫ ==============
DHT dht(DHTPIN, DHTTYPE);
WiFiManager wm;
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

// ============== BMP180 КАЛИБРОВКА ==============
int16_t AC1, AC2, AC3, B1_, B2_, MB, MC, MD;
uint16_t AC4, AC5, AC6;
bool bmpOk = false;

// ============== ПЕРЕМЕННЫЕ ==============
unsigned long lastRead = 0;
unsigned long lastMqttReconnect = 0;
char mqttClientId[32];

// ============== BMP180 ==============
void bmpReadCalibration() {
  Wire.beginTransmission(BMP180_ADDR);
  Wire.write(0xAA);
  Wire.endTransmission();
  Wire.requestFrom(BMP180_ADDR, 22);
  AC1 = Wire.read() << 8 | Wire.read();
  AC2 = Wire.read() << 8 | Wire.read();
  AC3 = Wire.read() << 8 | Wire.read();
  AC4 = Wire.read() << 8 | Wire.read();
  AC5 = Wire.read() << 8 | Wire.read();
  AC6 = Wire.read() << 8 | Wire.read();
  B1_ = Wire.read() << 8 | Wire.read();
  B2_ = Wire.read() << 8 | Wire.read();
  MB  = Wire.read() << 8 | Wire.read();
  MC  = Wire.read() << 8 | Wire.read();
  MD  = Wire.read() << 8 | Wire.read();
}

float bmpReadPressure() {
  Wire.beginTransmission(BMP180_ADDR);
  Wire.write(0xF4);
  Wire.write(0x2E);
  Wire.endTransmission();
  delay(5);

  Wire.beginTransmission(BMP180_ADDR);
  Wire.write(0xF6);
  Wire.endTransmission();
  Wire.requestFrom(BMP180_ADDR, 2);
  int32_t UT = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(BMP180_ADDR);
  Wire.write(0xF4);
  Wire.write(0x34 | (3 << 6));
  Wire.endTransmission();
  delay(26);

  Wire.beginTransmission(BMP180_ADDR);
  Wire.write(0xF6);
  Wire.endTransmission();
  Wire.requestFrom(BMP180_ADDR, 3);
  int32_t UP = ((Wire.read() << 16) | (Wire.read() << 8) | Wire.read()) >> (8 - 3);

  int32_t X1 = ((UT - (int32_t)AC6) * (int32_t)AC5) >> 15;
  int32_t X2 = ((int32_t)MC << 11) / (X1 + MD);
  int32_t B5 = X1 + X2;
  int32_t B6 = B5 - 4000;
  X1 = ((int32_t)B2_ * (B6 * B6 >> 12)) >> 11;
  X2 = ((int32_t)AC2 * B6) >> 11;
  int32_t X3 = X1 + X2;
  int32_t B3 = ((((int32_t)AC1 * 4 + X3) << 3) + 2) >> 2;
  X1 = ((int32_t)AC3 * B6) >> 13;
  X2 = ((int32_t)B1_ * ((B6 * B6) >> 12)) >> 16;
  X3 = ((X1 + X2) + 2) >> 2;
  uint32_t B4 = ((uint32_t)AC4 * (uint32_t)(X3 + 32768)) >> 15;
  uint32_t B7 = ((uint32_t)UP - B3) * (uint32_t)(50000UL >> 3);
  int32_t p;
  if (B7 < 0x80000000) {
    p = (B7 * 2) / B4;
  } else {
    p = (B7 / B4) * 2;
  }
  X1 = (p >> 8) * (p >> 8);
  X1 = (X1 * 3038) >> 16;
  X2 = (-7357 * p) >> 16;
  p = p + ((X1 + X2 + 3791) >> 4);

  return p / 133.322;
}

// ============== СБОРКА JSON ==============
void buildJson(char* buffer, size_t size) {
  float temp    = dht.readTemperature();
  float hum     = dht.readHumidity();
  bool  dhtOk   = !(isnan(temp) || isnan(hum));

  float press   = 0.0;
  bool  pressOk = false;
  if (bmpOk) {
    press = bmpReadPressure();
    pressOk = (press > 0);
  }

  int pos = 0;
  pos += snprintf(buffer + pos, size - pos, "{");

  if (dhtOk) {
    pos += snprintf(buffer + pos, size - pos, "\"temp\":%.1f", temp);
  } else {
    pos += snprintf(buffer + pos, size - pos, "\"temp\":null");
  }

  if (dhtOk) {
    pos += snprintf(buffer + pos, size - pos, ", \"hum\":%.1f", hum);
  } else {
    pos += snprintf(buffer + pos, size - pos, ", \"hum\":null");
  }

  if (pressOk) {
    pos += snprintf(buffer + pos, size - pos, ", \"press\":%.1f", press);
  } else {
    pos += snprintf(buffer + pos, size - pos, ", \"press\":null");
  }

  pos += snprintf(buffer + pos, size - pos, ", \"uptime\":%lu", millis() / 1000);
  pos += snprintf(buffer + pos, size - pos, ", \"rssi\":%d", WiFi.RSSI());
  pos += snprintf(buffer + pos, size - pos, "}");
}

// ============== MQTT: ПЕРЕПОДКЛЮЧЕНИЕ ==============
void mqttReconnect() {
  if (mqtt.connected()) return;

  unsigned long now = millis();
  if (now - lastMqttReconnect < 5000) return;
  lastMqttReconnect = now;

  Serial.print("[MQTT] Connecting... ");

  if (mqtt.connect(mqttClientId,
                   NULL, NULL,
                   "dacha/status", 0, true,
                   "{\"status\":\"offline\"}")) {

    Serial.println("OK");

    // Online
    mqtt.publish("dacha/status", "{\"status\":\"online\"}", true);

    // Auto-discovery
    char discovery[256];
    snprintf(discovery, sizeof(discovery),
      "{\"device_id\":\"%s\","
      "\"device_type\":\"weather_station\","
      "\"sensors\":[\"temperature\",\"humidity\",\"pressure\"],"
      "\"model\":\"esp32-weather-v0.3\"}",
      mqttClientId);
    mqtt.publish("dacha/discovery", discovery, true);

    Serial.print("[MQTT] Published discovery: ");
    Serial.println(discovery);

  } else {
    Serial.print("FAILED (rc=");
    Serial.print(mqtt.state());
    Serial.println(")");
  }
}

// ============== SETUP ==============
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("=== Dacha Weather Station v0.3 ===");

  // --- Инициализация датчиков ---
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.println("[I2C] OK");

  dht.begin();
  Serial.println("[AM2302] OK");

  Wire.beginTransmission(BMP180_ADDR);
  if (Wire.endTransmission() == 0) {
    bmpReadCalibration();
    bmpOk = true;
    Serial.println("[BMP180] OK");
  } else {
    Serial.println("[BMP180] NOT FOUND");
  }

  // --- Wi-Fi Manager ---
  WiFi.mode(WIFI_STA);
  wm.setConfigPortalBlocking(false);
  wm.setAPCallback([](WiFiManager* mgr) {
    Serial.println("[WiFi] AP mode: DachaWeather-Setup / dacha1234");
  });
  wm.setConfigPortalTimeout(300);

  if (!wm.autoConnect("DachaWeather-Setup", "dacha1234")) {
    Serial.println("[WiFi] Starting config portal...");
  } else {
    Serial.println("[WiFi] Connected!");
    Serial.print("[WiFi] IP: ");
    Serial.println(WiFi.localIP());
  }

  // --- MQTT ---
  snprintf(mqttClientId, sizeof(mqttClientId), "DachaWeather_%06X", 
           (uint32_t)(ESP.getEfuseMac() & 0xFFFFFF));
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  Serial.print("[MQTT] Client ID: ");
  Serial.println(mqttClientId);
  Serial.print("[MQTT] Broker: ");
  Serial.print(MQTT_BROKER);
  Serial.print(":");
  Serial.println(MQTT_PORT);

  Serial.println();
}

// ============== LOOP ==============
// ============== LOOP (исправленный интервал) ==============
void loop() {
  wm.process();

  // MQTT — обслуживаем всегда
  if (WiFi.status() == WL_CONNECTED) {
    mqttReconnect();
    mqtt.loop();
  }

  // Опрос датчиков — СТРОГО по интервалу, без дрейфа
  unsigned long now = millis();
  if (now - lastRead >= READ_INTERVAL) {
    lastRead += READ_INTERVAL;  // <-- прибавляем, а не присваиваем now!

    // Если по какой-то причине отстали больше чем на 2 интервала — догоняем
    if (now - lastRead > READ_INTERVAL) {
      lastRead = now;
    }

    char buffer[JSON_BUFFER_SIZE];
    buildJson(buffer, sizeof(buffer));
    Serial.println(buffer);

    if (mqtt.connected()) {
      mqtt.publish("dacha/weather", buffer);
    } else {
      Serial.println("[MQTT] Not connected");
    }
  }
}