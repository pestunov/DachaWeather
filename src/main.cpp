#include <Arduino.h>
#include <Wire.h>
#include <DHT.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <WiFiClientSecure.h>
#include <Update.h>           // ← OTA
#include <HTTPClient.h>       // ← для скачивания прошивки

// ============== КОНФИГУРАЦИЯ ==============
#define DHTPIN          4
#define DHTTYPE         DHT22
#define I2C_SDA         21
#define I2C_SCL         22
#define BMP180_ADDR     0x77
#define JSON_BUFFER_SIZE 128
#define READ_INTERVAL   10000      // 10 секунд

// MQTT
#define MQTT_BROKER     "192.168.1.107"  // IP БРОКЕРА!
#define MQTT_PORT       8883
#define MQTT_USER       "dacha"
#define MQTT_PASSWORD   ""  // после первой прошивки сменить на ""

// Fail-safe: потеря связи > 5 минут → тревога
#define FAILSAFE_TIMEOUT 300000  // 5 минут в миллисекундах

// TLS: CA-сертификат (зашит в коде, это НЕ секрет)
static const char CA_CERT[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDqzCCApOgAwIBAgIUchwwJCu3FqMYOUUiyCUWKaIf1pgwDQYJKoZIhvcNAQEL
BQAwZTELMAkGA1UEBhMCUlUxEzARBgNVBAgMClNvbWUtU3RhdGUxDjAMBgNVBAcM
BVRvbXNrMSEwHwYDVQQKDBhJbnRlcm5ldCBXaWRnaXRzIFB0eSBMdGQxDjAMBgNV
BAMMBWRhY2hhMB4XDTI2MDYxOTE5MDIwNFoXDTM2MDYxNjE5MDIwNFowZTELMAkG
A1UEBhMCUlUxEzARBgNVBAgMClNvbWUtU3RhdGUxDjAMBgNVBAcMBVRvbXNrMSEw
HwYDVQQKDBhJbnRlcm5ldCBXaWRnaXRzIFB0eSBMdGQxDjAMBgNVBAMMBWRhY2hh
MIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEA6XWe9WL57g3vT/FRqzdU
zUGJ+TtziDmJBHshKcb58C5/JGKQbp3q0CAwrtQuz34xxUjeM67sA0wWXTtuyNJ1
JgSSKcW5y247F2bY77iOPP7HKr+6hDyo+ehYT5GaRpDTnaaBWBpZLGm1X8d8aiwS
93qAgoAbohaMksv+rT7ACU1uC7/vdgR7gS6PWEOBLMdnrBKOBiv+UO5SYnPzyh7Q
eI1hKWXyvc7pe+EjTxDdF93wM+hjeQoTMFDLIOOIsnR02HDwBIK4j9AUgD3oTrgo
OSKWJLlxSz6sOgxDOOH95YZj5CuruhYQ/OPGfgeLbc0Ouh2UmbhkAuX5WMIJGS+w
sQIDAQABo1MwUTAdBgNVHQ4EFgQURaIVwfgD/ymbYp3p4OLgxtNO9ewwHwYDVR0j
BBgwFoAURaIVwfgD/ymbYp3p4OLgxtNO9ewwDwYDVR0TAQH/BAUwAwEB/zANBgkq
hkiG9w0BAQsFAAOCAQEANPWHfQNqE8jgKx48XFYZA4jCO7yy6j72CMkuS9cYJV4K
dR68kF0uSO+/QN7OUJ6XgomL4FJR0J9prrFDV+AWaTc1mBkKE5x2cEZj+bxoFP33
IL6eZQYZ2v4mmExLAzSajyBtOz9JI0D5Q93Qt4qfk9rCivBGKVCETEkIvwKG7KAv
mVgY7Z8jSVxMfkhaXfK3D7xNMsHGXfNgKe6b+HLLHT8cES5Fykwuoz6T+s0SuPdL
4OLTRkW7xLT855GNO4FB4lZMJ5MkzrKzrUFK34LeEXJVFhWdRnt6N0qX5mApJef6
bgekEk3kQahemzDBQYFIBBFnivN4PVuDZgf9oclQbA==
-----END CERTIFICATE-----
)EOF";

// ============== ОБЪЕКТЫ ==============
DHT dht(DHTPIN, DHTTYPE);
WiFiManager wm;
WiFiClientSecure wifiClient;
PubSubClient mqtt(wifiClient);

// ============== BMP180 КАЛИБРОВКА ==============
int16_t AC1, AC2, AC3, B1_, B2_, MB, MC, MD;
uint16_t AC4, AC5, AC6;
bool bmpOk = false;

// ============== ПЕРЕМЕННЫЕ ==============
unsigned long lastRead = 0;
unsigned long lastMqttReconnect = 0;
unsigned long lastMqttOk = 0;         // когда был последний успешный MQTT-обмен
bool failsafeTriggered = false;
char mqttClientId[32];
String mqttPassword;

// OTA
bool otaInProgress = false;

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

float bmpReadPressure(bool& ok) {
  ok = false;  // по умолчанию — ошибка

  // Запрос температуры
  Wire.beginTransmission(BMP180_ADDR);
  Wire.write(0xF4);
  Wire.write(0x2E);
  if (Wire.endTransmission() != 0) return 0;
  delay(5);

  Wire.beginTransmission(BMP180_ADDR);
  Wire.write(0xF6);
  if (Wire.endTransmission() != 0) return 0;
  if (Wire.requestFrom(BMP180_ADDR, 2) != 2) return 0;
  int32_t UT = Wire.read() << 8 | Wire.read();

  // Запрос давления
  Wire.beginTransmission(BMP180_ADDR);
  Wire.write(0xF4);
  Wire.write(0x34 | (3 << 6));
  if (Wire.endTransmission() != 0) return 0;
  delay(26);

  Wire.beginTransmission(BMP180_ADDR);
  Wire.write(0xF6);
  if (Wire.endTransmission() != 0) return 0;
  if (Wire.requestFrom(BMP180_ADDR, 3) != 3) return 0;
  int32_t UP = ((Wire.read() << 16) | (Wire.read() << 8) | Wire.read()) >> (8 - 3);

  // Расчёт
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

  ok = true;  // успех
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
    press = bmpReadPressure(pressOk);  // ← теперь с параметром
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

// ============== ОТПРАВКА ACK/NACK ==============
void sendAck(const char* cmd, bool success, const char* reason = nullptr) {
  char response[128];
  if (success) {
    snprintf(response, sizeof(response), "{\"cmd\":\"%s\",\"result\":\"ack\"}", cmd);
  } else {
    snprintf(response, sizeof(response), "{\"cmd\":\"%s\",\"result\":\"nack\",\"reason\":\"%s\"}", cmd, reason ? reason : "unknown");
  }
  mqtt.publish("dacha/command/response", response);
  Serial.print("[CMD] Response: ");
  Serial.println(response);
}

// ============== OTA: СКАЧИВАНИЕ И УСТАНОВКА ==============
void performOTA(const char* url) {
  Serial.print("[OTA] Downloading: ");
  Serial.println(url);

  HTTPClient http;
  http.begin(url);
  int httpCode = http.GET();

  if (httpCode != 200) {
    Serial.printf("[OTA] HTTP error: %d\n", httpCode);
    sendAck("OTA_UPDATE", false, "HTTP download failed");
    http.end();
    return;
  }

  int contentLength = http.getSize();
  Serial.printf("[OTA] Size: %d bytes\n", contentLength);

  if (contentLength <= 0) {
    Serial.println("[OTA] Empty response");
    sendAck("OTA_UPDATE", false, "empty response");
    http.end();
    return;
  }

  // Начинаем OTA
  if (!Update.begin(contentLength)) {
    Serial.printf("[OTA] Begin failed: %s\n", Update.errorString());
    sendAck("OTA_UPDATE", false, Update.errorString());
    http.end();
    return;
  }

  // Пишем потоком
  WiFiClient* stream = http.getStreamPtr();
  size_t written = Update.writeStream(*stream);

  if (written != contentLength) {
    Serial.printf("[OTA] Write failed: %d/%d\n", written, contentLength);
    sendAck("OTA_UPDATE", false, "write failed");
    http.end();
    return;
  }

  if (!Update.end()) {
    Serial.printf("[OTA] End failed: %s\n", Update.errorString());
    sendAck("OTA_UPDATE", false, Update.errorString());
    http.end();
    return;
  }

  if (!Update.isFinished()) {
    Serial.println("[OTA] Not finished");
    sendAck("OTA_UPDATE", false, "not finished");
    http.end();
    return;
  }

  http.end();
  Serial.println("[OTA] SUCCESS. Rebooting in 2 seconds...");
  sendAck("OTA_UPDATE", true);

  delay(2000);
  ESP.restart();
}

// ============== ОБРАБОТКА КОМАНД ==============
void handleCommand(const char* cmd, const char* payload) {
  Serial.print("[CMD] Received: ");
  Serial.println(payload);

  // --- RESTART ---
  if (strcmp(cmd, "RESTART") == 0) {
    sendAck("RESTART", true);
    delay(100);  // дать уйти ack
    ESP.restart();
  }

  // --- STATUS ---
  else if (strcmp(cmd, "STATUS") == 0) {
    char status[256];
    snprintf(status, sizeof(status),
      "{\"uptime\":%lu,\"rssi\":%d,\"free_heap\":%u,\"mqtt_connected\":%s,\"failsafe\":%s}",
      millis() / 1000,
      WiFi.RSSI(),
      ESP.getFreeHeap(),
      mqtt.connected() ? "true" : "false",
      failsafeTriggered ? "true" : "false");
    mqtt.publish("dacha/status", status);
    sendAck("STATUS", true);
  }

  // --- OTA_UPDATE ---
  else if (strcmp(cmd, "OTA_UPDATE") == 0) {
    // Ищем URL в payload: {"cmd":"OTA_UPDATE","url":"http://..."}
    const char* urlStart = strstr(payload, "\"url\"");
    if (urlStart) {
      urlStart = strchr(urlStart, ':');
      if (urlStart) {
        urlStart++;
        while (*urlStart == ' ' || *urlStart == '"') urlStart++;
        char url[256];
        int i = 0;
        while (*urlStart && *urlStart != '"' && i < 255) {
          url[i++] = *urlStart++;
        }
        url[i] = '\0';
        Serial.printf("[CMD] OTA URL: %s\n", url);
        performOTA(url);
        return;
      }
    }
    sendAck("OTA_UPDATE", false, "missing url");
  }

  // --- Неизвестная команда ---
  else {
    sendAck(cmd, false, "unknown command");
  }
}

// ============== MQTT CALLBACK ==============
void mqttCallback(char* topic, byte* message, unsigned int length) {
  // Пришло сообщение — сбрасываем таймер fail-safe
  lastMqttOk = millis();

  // Преобразуем payload в строку
  char payload[128];
  unsigned int len = length < 127 ? length : 127;
  memcpy(payload, message, len);
  payload[len] = '\0';

  Serial.print("[MQTT] Message: ");
  Serial.print(topic);
  Serial.print(" → ");
  Serial.println(payload);

  // Парсим JSON: ищем "cmd"
  // Простой парсер без библиотек — ищем ключ "cmd"
  char cmd[32] = "";
  const char* cmdStart = strstr(payload, "\"cmd\"");
  if (cmdStart) {
    cmdStart = strchr(cmdStart, ':');
    if (cmdStart) {
      cmdStart++; // пропускаем ':'
      while (*cmdStart == ' ' || *cmdStart == '"') cmdStart++;
      int i = 0;
      while (*cmdStart && *cmdStart != '"' && *cmdStart != ' ' && i < 31) {
        cmd[i++] = *cmdStart++;
      }
      cmd[i] = '\0';
    }
  }

  if (strlen(cmd) > 0) {
    handleCommand(cmd, payload);
  } else {
    sendAck("unknown", false, "no cmd field");
  }
}

// ============== MQTT: ПЕРЕПОДКЛЮЧЕНИЕ ==============
void mqttReconnect() {
  if (mqtt.connected()) return;

  unsigned long now = millis();
  if (now - lastMqttReconnect < 5000) return;
  lastMqttReconnect = now;

  Serial.print("[MQTT] Connecting TLS... ");

  if (mqtt.connect(mqttClientId,
                   MQTT_USER, mqttPassword.c_str(),
                   "dacha/status", 0, true,
                   "{\"status\":\"offline\"}")) {

    Serial.println("OK");
    lastMqttOk = millis();  // сброс таймера fail-safe

    mqtt.publish("dacha/status", "{\"status\":\"online\"}", true);

    char discovery[256];
    snprintf(discovery, sizeof(discovery),
      "{\"device_id\":\"%s\","
      "\"device_type\":\"weather_station\","
      "\"sensors\":[\"temperature\",\"humidity\",\"pressure\"],"
      "\"commands\":[\"RESTART\",\"STATUS\",\"OTA_UPDATE\"],"
      "\"model\":\"esp32-weather-v0.61\"}",
      mqttClientId);
    mqtt.publish("dacha/discovery", discovery, true);

    Serial.print("[MQTT] Discovery: ");
    Serial.println(discovery);

    // --- ПОДПИСКА НА КОМАНДЫ ---
    mqtt.subscribe("dacha/command");
    Serial.println("[MQTT] Subscribed to dacha/command");

  } else {
    Serial.print("FAILED (rc=");
    Serial.print(mqtt.state());
    Serial.println(")");
  }
}

// ============== FAIL-SAFE ПРОВЕРКА ==============
void checkFailsafe() {
  if (!mqtt.connected()) return;  // нет связи — не считаем

  unsigned long now = millis();
  if (now - lastMqttOk > FAILSAFE_TIMEOUT) {
    if (!failsafeTriggered) {
      failsafeTriggered = true;
      Serial.println("[FAILSAFE] TRIGGERED! Connection lost > 5 min");
      mqtt.publish("dacha/status", "{\"status\":\"failsafe\"}", true);
      // Здесь в будущем: digitalWrite(RELAY_PIN, LOW);
    }
  } else {
    if (failsafeTriggered) {
      failsafeTriggered = false;
      Serial.println("[FAILSAFE] Cleared. Connection restored");
      mqtt.publish("dacha/status", "{\"status\":\"online\"}", true);
    }
  }
}

// ============== SETUP ==============
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("=== Dacha Weather Station v0.6 (OTA) ===");

  // --- Датчики ---
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

  // --- Пароль MQTT ---
  Preferences prefs;
  prefs.begin("mqtt", false);

  if (strlen(MQTT_PASSWORD) > 0) {
    prefs.putString("password", MQTT_PASSWORD);
    mqttPassword = MQTT_PASSWORD;
    Serial.println("[NVS] Password saved");
  } else {
    mqttPassword = prefs.getString("password", "");
    if (mqttPassword.isEmpty()) {
      Serial.println("[NVS] WARNING: No password!");
    } else {
      Serial.println("[NVS] Password loaded from NVS");
    }
  }
  prefs.end();

  // --- Wi-Fi ---
  WiFi.mode(WIFI_STA);
  wm.setConfigPortalBlocking(false);
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
  wifiClient.setCACert(CA_CERT);
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  Serial.print("[MQTT] Client: "); Serial.println(mqttClientId);
  Serial.println();
}

// ============== LOOP ==============
void loop() {
  wm.process();

  if (WiFi.status() == WL_CONNECTED) {
    mqttReconnect();
    mqtt.loop();
  }

  // Fail-safe проверка
  checkFailsafe();

  // Опрос датчиков
  unsigned long now = millis();
  if (now - lastRead >= READ_INTERVAL) {
    lastRead += READ_INTERVAL;
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

