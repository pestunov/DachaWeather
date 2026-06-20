# Dacha Weather Station

Метеостанция на базе ESP32 с передачей данных по Wi-Fi (MQTT в будущем).

## Железо

| Компонент | Модель |
|-----------|--------|
| Микроконтроллер | ESP32-WROOM-32 (38 pin DevKit) |
| Температура и влажность | AM2302 (DHT22) |
| Атмосферное давление | BMP180 |
| Питание | USB 5V / внешний 5V через VIN |

### Распиновка

| Датчик | Пин ESP32 | Протокол |
|--------|-----------|----------|
| AM2302 DATA | GPIO 4 | OneWire |
| BMP180 SDA | GPIO 21 | I2C |
| BMP180 SCL | GPIO 22 | I2C |
| Питание датчиков | 3.3V + GND (один угол платы!) | — |

### Важно по питанию
Питание датчиков и ESP32 — от одной точки (один угол платы). Питание от USB-порта компьютера может быть недостаточным при пиковых нагрузках. Рекомендуется зарядное устройство 5V/1A+ или внешний стабилизатор.

## Возможности

- [x] Чтение температуры, влажности, давления
- [x] JSON-формат вывода (Serial Monitor)
- [x] Wi-Fi Manager с Captive Portal (настройка через телефон)
- [x] MQTT: публикация данных и приём команд, status, LWT
- [x] TLS-шифрование
- [ ] OTA-обновления
- [ ] Watchdog и fail-safe

## Сборка и прошивка

### Требования
- [VS Code](https://code.visualstudio.com/) + [PlatformIO IDE](https://platformio.org/install/ide?install=vscode)
- Драйверы USB-UART (CP210x или CH340 — зависит от платы)

### Сборка
```bash
# Клонировать репозиторий
git clone <repo-url>
cd DachaWeather

# PlatformIO сам скачает зависимости и соберёт проект
pio run
```

### Прошивка
```bash
pio run --target upload
```

### Монитор порта
```bash
pio device monitor --baud 115200
```
## Настройка Wi-Fi (первый запуск)

- После прошивки плата создаёт точку доступа DachaWeather-Setup   
- Пароль: dacha1234   
- Подключиться с телефона/ноутбука к этой сети
- Открыть браузер → 192.168.4.1
- Выбрать домашнюю Wi-Fi сеть, ввести пароль
- Плата перезагрузится и подключится к сети
- Для сброса настроек: зажать кнопку BOOT на 5 секунд (будет реализовано позже) или стереть flash через pio run --target erase.

## Формат данных (JSON)
```json
{
  "temp": 23.5,
  "hum": 54.2,
  "press": 745.3,
  "uptime": 3600,
  "rssi": -55
}
```

## Как настроить сертификаты в Mosquitto

```text
sudo mkdir -p /etc/mosquitto/certs
cd /etc/mosquitto/certs

# 1. CA-ключ
sudo openssl genrsa -out ca.key 2048

# 2. CA-сертификат
sudo openssl req -new -x509 -days 3650 -key ca.key -out ca.crt
# Common Name: Dacha-CA (остальное произвольно)

# 3. Ключ брокера
sudo openssl genrsa -out broker.key 2048

# 4. Запрос на сертификат брокера
sudo openssl req -new -key broker.key -out broker.csr
# Common Name: 192.168.1.107 (IP брокера, остальное произвольно)

# 5. Подпись сертификата брокера
sudo openssl x509 -req -in broker.csr -CA ca.crt -CAkey ca.key -CAcreateserial -out broker.crt -days 3650

# 6. Права
sudo chown mosquitto:mosquitto /etc/mosquitto/certs/*
sudo chown mosquitto:mosquitto /etc/mosquitto/passwd
sudo chmod 644 /etc/mosquitto/certs/*.crt
sudo chmod 600 /etc/mosquitto/certs/*.key
sudo chmod 644 /etc/mosquitto/passwd

# и дериктория с сртификатами доступна
sudo chmod 755 /etc/mosquitto/certs

# 7. Конфиг (/etc/mosquitto/mosquitto.conf)
listener 1883 localhost
allow_anonymous true

listener 8883 0.0.0.0
cafile   /etc/mosquitto/certs/ca.crt
certfile /etc/mosquitto/certs/broker.crt
keyfile  /etc/mosquitto/certs/broker.key
require_certificate false

password_file /etc/mosquitto/passwd
allow_anonymous false

# 8. Перезапуск
sudo systemctl restart mosquitto
sudo systemctl status mosquitto

#9. Проверка TLS
mosquitto_sub -h 192.168.1.107 -p 8883 --cafile /etc/mosquitto/certs/ca.crt -u dacha -P ТВОЙ_ПАРОЛЬ -t "dacha/#" -v
```

## Дорожная карта
- v0.1 Датчики, JSON в Serial
- v0.2 Wi-Fi Manager с Captive Portal
- v0.3 MQTT-клиент, публикация на брокер
- v0.4 TLS, безопасное хранение паролей
- v0.5 Команды управления, fail-safe
- v0.6 OTA-обновления
- v1.0 Watchdog, диагностика, стабилизация
