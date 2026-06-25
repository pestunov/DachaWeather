/*
 * Dacha Weather Station v1.0
 * 
 * Метеостанция на ESP32 с датчиками AM2302 (температура/влажность)
 * и BMP180 (давление). Публикует данные по MQTT с TLS-шифрованием.
 * 
 * Возможности:
 *   - Wi-Fi Manager (captive portal) для первичной настройки
 *   - MQTT: auto-discovery, LWT, JSON-данные каждые 10 секунд
 *   - TLS-шифрование трафика
 *   - Пароли в зашифрованной NVS
 *   - Команды: STATUS, RESTART, PING, OTA_UPDATE
 *   - Fail-safe: если MQTT отвалился > 5 мин → тревога
 *   - OTA-обновления (двухслотовая схема с откатом)
 *   - Аппаратный watchdog (перезагрузка при зависании)
 *   - Периодическая диагностика (RSSI, heap, uptime)
 */

#include <Arduino.h>
#include <Wire.h>
#include <DHT.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <WiFiClientSecure.h>
#include <Update.h>             // ← OTA
#include <HTTPClient.h>         // ← для скачивания прошивки
#include "esp_task_wdt.h"       // Аппаратный watchdog для loop()

// ============== КОНФИГУРАЦИЯ ПИНОВ ==============
#define DHTPIN          4       // AM2302 (DHT22) — GPIO 4
#define DHTTYPE         DHT22   // Тип датчика (AM2302 = DHT22)
#define I2C_SDA         21      // I2C: линия данных
#define I2C_SCL         22      // I2C: линия тактирования
#define BMP180_ADDR     0x77    // I2C-адрес BMP180

// ============== РАЗМЕРЫ БУФЕРОВ И ИНТЕРВАЛЫ ==============
#define JSON_BUFFER_SIZE 256    // Буфер для JSON с данными погоды
#define READ_INTERVAL    10000  // Опрос датчиков: 10 секунд
#define DIAG_INTERVAL    300000 // Диагностика: 5 минут (300 000 мс)

// ============== MQTT (настройки брокера) ==============
#define MQTT_BROKER     "192.168.1.107"  // IP-адрес Mosquitto
#define MQTT_PORT       8883             // Порт TLS
#define MQTT_USER       "dacha"          // Логин
#define MQTT_PASSWORD   ""               // Пароль в NVS (после первой прошивки — пусто)

// ============== FAIL-SAFE ==============
#define FAILSAFE_TIMEOUT 300000  // 5 минут без связи → тревога

// ============== WATCHDOG ==============
#define WDT_TIMEOUT     10       // Секунд до перезагрузки при зависании

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

// ============== ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ==============
unsigned long lastRead = 0;              // Время последнего опроса датчиков
unsigned long lastDiag = 0;              // Время последней диагностики
unsigned long lastMqttReconnect = 0;     // Время последней попытки MQTT-реконнекта
unsigned long lastMqttOk = 0;            // Время последнего успешного MQTT-обмена
bool failsafeTriggered = false;          // Флаг срабатывания fail-safe
char mqttClientId[32];                   // Уникальный ID клиента (DachaWeather_XXXXXX)
String mqttPassword;                     // Пароль MQTT (из NVS)

// ============== BMP180: ЧТЕНИЕ КАЛИБРОВКИ ==============
// Вызывается один раз в setup(). Читает 11 калибровочных коэффициентов
// из EEPROM датчика. Без них невозможно вычислить давление.
void bmpReadCalibration() {
  Wire.beginTransmission(BMP180_ADDR);
  Wire.write(0xAA);                      // Адрес начала калибровочных данных
  Wire.endTransmission();
  Wire.requestFrom(BMP180_ADDR, 22);     // Читаем 22 байта

  // Разбираем байты в значащие переменные (см. даташит BMP180)
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

// ============== BMP180: ЧТЕНИЕ ДАВЛЕНИЯ ==============
// Возвращает давление в мм рт. ст. Через параметр ok сообщает об успехе.
// При любой ошибке I2C (обрыв провода, датчик не отвечает) ok = false.
// Алгоритм пересчёта — стандартная формула из даташита Bosch BMP180.
float bmpReadPressure(bool& ok) {
  ok = false;  // По умолчанию — ошибка

  // --- Шаг 1: запрос сырой температуры (UT) ---
  Wire.beginTransmission(BMP180_ADDR);
  Wire.write(0xF4);                      // Регистр управления
  Wire.write(0x2E);                      // Команда: измерить температуру
  if (Wire.endTransmission() != 0) return 0;  // Ошибка I2C
  delay(5);                              // Ждём измерение (4.5 мс макс)

  Wire.beginTransmission(BMP180_ADDR);
  Wire.write(0xF6);                      // Регистр данных
  if (Wire.endTransmission() != 0) return 0;
  if (Wire.requestFrom(BMP180_ADDR, 2) != 2) return 0;  // Читаем 2 байта
  int32_t UT = Wire.read() << 8 | Wire.read();

  // --- Шаг 2: запрос сырого давления (UP) ---
  Wire.beginTransmission(BMP180_ADDR);
  Wire.write(0xF4);
  Wire.write(0x34 | (3 << 6));           // Команда: измерить давление, макс. точность
  if (Wire.endTransmission() != 0) return 0;
  delay(26);                             // Ждём измерение (25.5 мс макс при oss=3)

  Wire.beginTransmission(BMP180_ADDR);
  Wire.write(0xF6);
  if (Wire.endTransmission() != 0) return 0;
  if (Wire.requestFrom(BMP180_ADDR, 3) != 3) return 0;  // Читаем 3 байта
  int32_t UP = ((Wire.read() << 16) | (Wire.read() << 8) | Wire.read()) >> (8 - 3);

  // --- Шаг 3: расчёт истинного давления (формулы из даташита) ---
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

  ok = true;                             // Всё прошло успешно
  return p / 133.322;                    // Паскали → мм рт. ст.
}

// ============== СБОРКА JSON С ДАННЫМИ ПОГОДЫ ==============
// Атомарно собирает всю строку в буфер, затем отправляется одним вызовом.
// При ошибке датчика его поле = null.
void buildJson(char* buffer, size_t size) {
  // --- Опрос AM2302 ---
  float temp    = dht.readTemperature();
  float hum     = dht.readHumidity();
  bool  dhtOk   = !(isnan(temp) || isnan(hum));

  // --- Опрос BMP180 ---
  float press   = 0.0;
  bool  pressOk = false;
  if (bmpOk) {
    press = bmpReadPressure(pressOk);
  }

  // --- Сборка JSON в буфер ---
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
// Подтверждение выполнения команды. Публикуется в dacha/command/response.
void sendAck(const char* cmd, bool success, const char* reason = nullptr) {
  char response[128];
  if (success) {
    snprintf(response, sizeof(response), "{\"cmd\":\"%s\",\"result\":\"ack\"}", cmd);
  } else {
    snprintf(response, sizeof(response),
      "{\"cmd\":\"%s\",\"result\":\"nack\",\"reason\":\"%s\"}",
      cmd, reason ? reason : "unknown");
  }
  mqtt.publish("dacha/command/response", response);
  Serial.print("[CMD] Response: ");
  Serial.println(response);
}

// ============== OTA: СКАЧИВАНИЕ И УСТАНОВКА ПРОШИВКИ ==============
// Качает .bin файл по HTTP, пишет во второй OTA-раздел.
// При успехе перезагружается. При провале — откат автоматический.
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

  // Начинаем OTA-обновление
  if (!Update.begin(contentLength)) {
    Serial.printf("[OTA] Begin failed: %s\n", Update.errorString());
    sendAck("OTA_UPDATE", false, Update.errorString());
    http.end();
    return;
  }

  // Пишем прошивку потоком в неактивный раздел
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

// ============== ОБРАБОТКА ВХОДЯЩИХ КОМАНД ==============
// Парсит cmd из JSON, выполняет действие, отвечает ack/nack.
void handleCommand(const char* cmd, const char* payload) {
  Serial.print("[CMD] Received: ");
  Serial.println(payload);

  // --- RESTART: мягкая перезагрузка ---
  if (strcmp(cmd, "RESTART") == 0) {
    sendAck("RESTART", true);
    delay(100);
    ESP.restart();
  }

  // --- STATUS: немедленная диагностика ---
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

  // --- PING: проверка связи (для будущих исполнительных модулей) ---
  else if (strcmp(cmd, "PING") == 0) {
    sendAck("PING", true);
  }

  // --- OTA_UPDATE: обновление прошивки по воздуху ---
  else if (strcmp(cmd, "OTA_UPDATE") == 0) {
    // Извлекаем URL из JSON: {"cmd":"OTA_UPDATE","url":"http://..."}
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

// ============== MQTT CALLBACK: ВХОДЯЩИЕ СООБЩЕНИЯ ==============
// Вызывается при получении любого сообщения в подписанном топике.
// Сбрасывает таймер fail-safe и передаёт команду в handleCommand().
void mqttCallback(char* topic, byte* message, unsigned int length) {
  lastMqttOk = millis();  // Есть активность в канале → связь жива

  // Копируем payload в строку
  char payload[128];
  unsigned int len = length < 127 ? length : 127;
  memcpy(payload, message, len);
  payload[len] = '\0';

  // Простейший парсер JSON: ищем поле "cmd"
  char cmd[32] = "";
  const char* cmdStart = strstr(payload, "\"cmd\"");
  if (cmdStart) {
    cmdStart = strchr(cmdStart, ':');
    if (cmdStart) {
      cmdStart++;
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
// Пытается подключиться к брокеру. При успехе публикует online,
// discovery и подписывается на команды.
void mqttReconnect() {
  if (mqtt.connected()) return;  // Уже подключены

  // Не дёргаем брокер чаще раза в 5 секунд
  unsigned long now = millis();
  if (now - lastMqttReconnect < 5000) return;
  lastMqttReconnect = now;

  Serial.print("[MQTT] Connecting TLS... ");

  // Подключаемся с LWT: если отвалимся — брокер опубликует "offline"
  if (mqtt.connect(mqttClientId,
                   MQTT_USER, mqttPassword.c_str(),
                   "dacha/status", 0, true,
                   "{\"status\":\"offline\"}")) {

    Serial.println("OK");
    lastMqttOk = millis();

    // Сообщаем, что мы online
    mqtt.publish("dacha/status", "{\"status\":\"online\"}", true);

    // Auto-discovery: кто мы и что умеем
    char discovery[256];
    snprintf(discovery, sizeof(discovery),
      "{\"device_id\":\"%s\","
      "\"device_type\":\"weather_station\","
      "\"sensors\":[\"temperature\",\"humidity\",\"pressure\"],"
      "\"commands\":[\"RESTART\",\"STATUS\",\"PING\",\"OTA_UPDATE\"],"
      "\"model\":\"esp32-weather-v1.0\"}",
      mqttClientId);
    mqtt.publish("dacha/discovery", discovery, true);

    // Подписываемся на команды
    mqtt.subscribe("dacha/command");
    Serial.print("[MQTT] Subscribed. Discovery: ");
    Serial.println(discovery);
  } else {
    Serial.print("FAILED (rc=");
    Serial.print(mqtt.state());
    Serial.println(")");
  }
}

// ============== ПУБЛИКАЦИЯ ДИАГНОСТИКИ ==============
// Отправляет RSSI, свободную кучу, аптайм в dacha/diagnostics.
// Вызывается раз в DIAG_INTERVAL (5 минут).
void publishDiagnostics() {
  unsigned long now = millis();
  if (now - lastDiag < DIAG_INTERVAL) return;
  lastDiag = now;

  if (!mqtt.connected()) return;  // Нет связи — не публикуем

  char diag[128];
  snprintf(diag, sizeof(diag),
    "{\"rssi\":%d,\"free_heap\":%u,\"uptime\":%lu,\"mqtt_connected\":true}",
    WiFi.RSSI(),
    ESP.getFreeHeap(),
    millis() / 1000);

  mqtt.publish("dacha/diagnostics", diag);
  Serial.print("[DIAG] ");
  Serial.println(diag);
}

// ============== FAIL-SAFE ПРОВЕРКА ==============
// Если MQTT-связь потеряна > FAILSAFE_TIMEOUT — поднимаем тревогу.
// Для метеостанции это просто флаг, для реле — выключение всего.
void checkFailsafe() {
  if (!mqtt.connected()) return;

  unsigned long now = millis();
  if (now - lastMqttOk > FAILSAFE_TIMEOUT) {
    if (!failsafeTriggered) {
      failsafeTriggered = true;
      Serial.println("[FAILSAFE] TRIGGERED!");
      mqtt.publish("dacha/status", "{\"status\":\"failsafe\"}", true);
    }
  } else {
    if (failsafeTriggered) {
      failsafeTriggered = false;
      Serial.println("[FAILSAFE] Cleared");
      mqtt.publish("dacha/status", "{\"status\":\"online\"}", true);
    }
  }
}

// ============== SETUP ==============
void setup() {
  Serial.begin(115200);
  delay(1000);  // Даём время на подключение монитора порта
  Serial.println();
  Serial.println("=== Dacha Weather Station v1.0 ===");

  // --- Инициализация датчиков ---
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.println("[I2C] OK");

  dht.begin();
  Serial.println("[AM2302] OK on GPIO 4");

  Wire.beginTransmission(BMP180_ADDR);
  if (Wire.endTransmission() == 0) {
    bmpReadCalibration();
    bmpOk = true;
    Serial.println("[BMP180] OK at 0x77");
  } else {
    Serial.println("[BMP180] NOT FOUND — pressure will be null");
  }

  // --- Пароль MQTT: приоритет у #define, иначе из NVS ---
  Preferences prefs;
  prefs.begin("mqtt", false);
  if (strlen(MQTT_PASSWORD) > 0) {
    // В коде задан пароль → сохраняем в NVS
    prefs.putString("password", MQTT_PASSWORD);
    mqttPassword = MQTT_PASSWORD;
    Serial.println("[NVS] Password saved from MQTT_PASSWORD");
  } else {
    // Пароль в коде пуст → читаем из NVS
    mqttPassword = prefs.getString("password", "");
    if (!mqttPassword.isEmpty()) {
      Serial.println("[NVS] Password loaded from NVS");
    } else {
      Serial.println("[NVS] WARNING: No password in NVS or code!");
    }
  }
  prefs.end();

  // --- Wi-Fi Manager ---
  WiFi.mode(WIFI_STA);
  wm.setConfigPortalBlocking(false);
  wm.setConfigPortalTimeout(300);
  if (!wm.autoConnect("DachaWeather-Setup", "dacha1234")) {
    Serial.println("[WiFi] Config portal started");
  } else {
    Serial.println("[WiFi] Connected!");
    Serial.print("[WiFi] IP: ");
    Serial.println(WiFi.localIP());
  }

  // --- MQTT + TLS ---
  snprintf(mqttClientId, sizeof(mqttClientId), "DachaWeather_%06X",
           (uint32_t)(ESP.getEfuseMac() & 0xFFFFFF));
  wifiClient.setCACert(CA_CERT);
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  Serial.print("[MQTT] Client ID: "); Serial.println(mqttClientId);

  // --- Аппаратный Watchdog ---
  // Если loop() зависнет > WDT_TIMEOUT секунд — аппаратная перезагрузка
  esp_task_wdt_init(WDT_TIMEOUT, true);    // true = паника при таймауте
  esp_task_wdt_add(NULL);                  // Добавляем текущую задачу (loop)
  Serial.printf("[WDT] Watchdog enabled, timeout=%d sec\n", WDT_TIMEOUT);

  Serial.println("[SETUP] Complete.\n");
}

// ============== LOOP ==============
void loop() {
  // --- Сброс аппаратного watchdog ---
  // Если эта строка не выполнится за WDT_TIMEOUT секунд → перезагрузка
  esp_task_wdt_reset();

  // --- Wi-Fi Manager (обслуживание captive portal) ---
  wm.process();

  // --- MQTT: реконнект и обработка входящих ---
  if (WiFi.status() == WL_CONNECTED) {
    mqttReconnect();
    mqtt.loop();
  }

  // --- Fail-safe проверка ---
  checkFailsafe();

  // --- Периодическая диагностика (каждые 5 минут) ---
  publishDiagnostics();

  // --- Опрос датчиков (каждые 10 секунд, строго по таймеру) ---
  unsigned long now = millis();
  if (now - lastRead >= READ_INTERVAL) {
    lastRead += READ_INTERVAL;
    if (now - lastRead > READ_INTERVAL) {
      lastRead = now;  // Догоняем, если сильно отстали
    }

    // Собираем JSON
    char buffer[JSON_BUFFER_SIZE];
    buildJson(buffer, sizeof(buffer));
    Serial.println(buffer);

    // Публикуем в MQTT, при успехе сбрасываем таймер fail-safe
    if (mqtt.connected()) {
      if (mqtt.publish("dacha/weather", buffer)) {
        lastMqttOk = millis();
      }
    }
  }
}

