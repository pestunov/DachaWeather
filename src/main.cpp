#include <Arduino.h>
#include <Wire.h>
#include <DHT.h>

// ============== КОНФИГУРАЦИЯ ПИНОВ ==============
#define DHTPIN      4
#define DHTTYPE     DHT22
#define I2C_SDA     21
#define I2C_SCL     22
#define BMP180_ADDR 0x77

// ============== КОНФИГУРАЦИЯ ПРИЛОЖЕНИЯ ==============
#define JSON_BUFFER_SIZE 128

// ============== ИНТЕРВАЛ ОПРОСА ==============
const unsigned long READ_INTERVAL = 10000;

// ============== ОБЪЕКТЫ ==============
DHT dht(DHTPIN, DHTTYPE);

// ============== BMP180 КАЛИБРОВОЧНЫЕ КОЭФФИЦИЕНТЫ ==============
int16_t AC1, AC2, AC3, B1_, B2_, MB, MC, MD;
uint16_t AC4, AC5, AC6;
bool bmpOk = false;

// ============== ПЕРЕМЕННЫЕ ==============
unsigned long lastRead = 0;

// ============== BMP180: ЧТЕНИЕ КАЛИБРОВКИ ==============
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

// ============== BMP180: ЧТЕНИЕ ДАВЛЕНИЯ В мм рт. ст. ==============
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

// ============== SETUP ==============
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("=== Dacha Weather Station v0.1 ===");

  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.println("[I2C] OK (SDA=21, SCL=22)");

  dht.begin();
  Serial.println("[AM2302] OK (GPIO 4)");

  Wire.beginTransmission(BMP180_ADDR);
  if (Wire.endTransmission() == 0) {
    bmpReadCalibration();
    bmpOk = true;
    Serial.println("[BMP180] OK at 0x77");
  } else {
    Serial.println("[BMP180] NOT FOUND");
  }
  Serial.println();
}

// ============== LOOP ==============
void loop() {
  unsigned long now = millis();
  if (now - lastRead < READ_INTERVAL) return;
  lastRead = now;

  // --- ЭТАП 1: ОПРОС ВСЕХ ДАТЧИКОВ ---
  float temp     = dht.readTemperature();
  float hum      = dht.readHumidity();
  bool  dhtOk    = !(isnan(temp) || isnan(hum));

  float press    = 0.0;
  bool  pressOk  = false;
  if (bmpOk) {
    press = bmpReadPressure();
    pressOk = (press > 0);
  }

  // --- ЭТАП 2: СБОРКА СТРОКИ В ПАМЯТИ ---
  char buffer[JSON_BUFFER_SIZE];  // буфер под JSON
  int pos = 0;

  pos += snprintf(buffer + pos, sizeof(buffer) - pos, "{");

  // Температура
  if (dhtOk) {
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "\"temp\":%.1f", temp);
  } else {
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "\"temp\":null");
  }

  // Влажность
  if (dhtOk) {
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, ", \"hum\":%.1f", hum);
  } else {
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, ", \"hum\":null");
  }

  // Давление
  if (pressOk) {
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, ", \"press\":%.1f", press);
  } else {
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, ", \"press\":null");
  }

  // Служебные поля
  pos += snprintf(buffer + pos, sizeof(buffer) - pos, ", \"uptime\":%lu", millis() / 1000);
  pos += snprintf(buffer + pos, sizeof(buffer) - pos, "}");

  // --- ЭТАП 3: ОДНИМ ВЫСТРЕЛОМ ВЫВОД ---
  Serial.println(buffer);
}