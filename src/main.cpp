#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <DHTesp.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <cmath>

#include "config.h"

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

DHTesp dht;
Adafruit_MPU6050 mpu;

String mqttClientId;

bool wifiWasConnected = false;
bool mpuReady = false;

bool temperatureValid = false;
bool distanceValid = false;
bool accelerationValid = false;

float temperatureC = NAN;
float humidityPercent = NAN;
float distanceCm = NAN;

float accelX = NAN;
float accelY = NAN;
float accelZ = NAN;

float accelerationMagnitude = NAN;
float accelerationDeviation = NAN;

// Dados da janela de publicação
float accelerationPeak = 0.0f;
uint32_t accelerationSamples = 0;
uint32_t windowStartedAt = 0;

uint32_t lastWifiAttempt = 0;
uint32_t lastMqttAttempt = 0;
uint32_t lastDhtRead = 0;
uint32_t lastDistanceRead = 0;
uint32_t lastAccelRead = 0;
uint32_t lastPublish = 0;

uint32_t sequenceNumber = 0;

void formatJsonNumber(
    char* destination,
    size_t size,
    float value,
    bool valid,
    uint8_t decimals = 2
) {
    if (valid && std::isfinite(value)) {
        snprintf(destination, size, "%.*f", decimals, value);
    } else {
        snprintf(destination, size, "null");
    }
}

void startWifi() {
    lastWifiAttempt = millis();

    Serial.printf("[WiFi] Conectando em %s...\n", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void maintainConnections() {
    const uint32_t now = millis();

    if (WiFi.status() != WL_CONNECTED) {
        if (wifiWasConnected) {
            wifiWasConnected = false;
            mqttClient.disconnect();
            Serial.println("[WiFi] Conexao perdida.");
        }

        if (now - lastWifiAttempt >= WIFI_RETRY_MS) {
            startWifi();
        }

        return;
    }

    if (!wifiWasConnected) {
        wifiWasConnected = true;

        Serial.print("[WiFi] Conectado. IP: ");
        Serial.println(WiFi.localIP());

        lastMqttAttempt = now - MQTT_RETRY_MS;
    }

    if (!mqttClient.connected()) {
        if (now - lastMqttAttempt >= MQTT_RETRY_MS) {
            lastMqttAttempt = now;

            Serial.printf(
                "[MQTT] Conectando em %s:%u...\n",
                MQTT_HOST,
                static_cast<unsigned int>(MQTT_PORT)
            );

            if (mqttClient.connect(mqttClientId.c_str())) {
                Serial.print("[MQTT] Conectado. Topico: ");
                Serial.println(MQTT_TOPIC);
            } else {
                Serial.printf(
                    "[MQTT] Falha. Estado: %d\n",
                    mqttClient.state()
                );
            }
        }

        return;
    }

    mqttClient.loop();
}

void readTemperature() {
    const uint32_t now = millis();

    if (now - lastDhtRead < DHT_INTERVAL_MS) {
        return;
    }

    lastDhtRead = now;

    const TempAndHumidity reading = dht.getTempAndHumidity();

    temperatureValid =
        std::isfinite(reading.temperature) &&
        std::isfinite(reading.humidity);

    if (temperatureValid) {
        temperatureC = reading.temperature;
        humidityPercent = reading.humidity;
    } else {
        temperatureC = NAN;
        humidityPercent = NAN;

        Serial.println("[DHT22] Leitura invalida.");
    }
}

void readDistance() {
    const uint32_t now = millis();

    if (now - lastDistanceRead < DISTANCE_INTERVAL_MS) {
        return;
    }

    lastDistanceRead = now;

    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);

    digitalWrite(TRIG_PIN, LOW);

    const unsigned long duration = pulseIn(
        ECHO_PIN,
        HIGH,
        ECHO_TIMEOUT_US
    );

    if (duration == 0) {
        distanceValid = false;
        distanceCm = NAN;
        return;
    }

    const float measuredDistance = duration / 58.0f;

    distanceValid =
        measuredDistance >= 2.0f &&
        measuredDistance <= 400.0f;

    distanceCm = distanceValid ? measuredDistance : NAN;
}

void readAcceleration() {
    const uint32_t now = millis();

    if (now - lastAccelRead < ACCEL_INTERVAL_MS) {
        return;
    }

    lastAccelRead = now;

    if (!mpuReady) {
        accelerationValid = false;
        return;
    }

    sensors_event_t acceleration;
    sensors_event_t gyro;
    sensors_event_t sensorTemperature;

    const bool success = mpu.getEvent(
        &acceleration,
        &gyro,
        &sensorTemperature
    );

    accelerationValid =
        success &&
        std::isfinite(acceleration.acceleration.x) &&
        std::isfinite(acceleration.acceleration.y) &&
        std::isfinite(acceleration.acceleration.z);

    if (!accelerationValid) {
        return;
    }

    accelX = acceleration.acceleration.x;
    accelY = acceleration.acceleration.y;
    accelZ = acceleration.acceleration.z;

    accelerationMagnitude = sqrtf(
        accelX * accelX +
        accelY * accelY +
        accelZ * accelZ
    );

    accelerationDeviation = fabsf(
        accelerationMagnitude - GRAVITY_MS2
    );

    if (accelerationDeviation > accelerationPeak) {
        accelerationPeak = accelerationDeviation;
    }

    accelerationSamples++;
}

void publishTelemetry() {
    const uint32_t now = millis();

    if (now - lastPublish < PUBLISH_INTERVAL_MS) {
        return;
    }

    lastPublish = now;
    sequenceNumber++;

    char tempText[24];
    char humidityText[24];
    char distanceText[24];
    char axText[24];
    char ayText[24];
    char azText[24];
    char magnitudeText[24];
    char deviationText[24];
    char peakText[24];

    formatJsonNumber(
        tempText, sizeof(tempText),
        temperatureC, temperatureValid
    );

    formatJsonNumber(
        humidityText, sizeof(humidityText),
        humidityPercent, temperatureValid
    );

    formatJsonNumber(
        distanceText, sizeof(distanceText),
        distanceCm, distanceValid
    );

    formatJsonNumber(
        axText, sizeof(axText),
        accelX, accelerationValid
    );

    formatJsonNumber(
        ayText, sizeof(ayText),
        accelY, accelerationValid
    );

    formatJsonNumber(
        azText, sizeof(azText),
        accelZ, accelerationValid
    );

    formatJsonNumber(
        magnitudeText, sizeof(magnitudeText),
        accelerationMagnitude, accelerationValid
    );

    formatJsonNumber(
        deviationText, sizeof(deviationText),
        accelerationDeviation, accelerationValid
    );

    formatJsonNumber(
        peakText, sizeof(peakText),
        accelerationPeak, accelerationSamples > 0
    );

    char payload[1200];

    const int length = snprintf(
        payload,
        sizeof(payload),
        "{"
          "\"grupo\":\"%s\","
          "\"dispositivo\":\"%s\","
          "\"uptime_ms\":%lu,"
          "\"sequencia\":%lu,"
          "\"janela_ms\":%lu,"
          "\"temperatura_valida\":%s,"
          "\"temperatura_c\":%s,"
          "\"umidade_pct\":%s,"
          "\"distancia_valida\":%s,"
          "\"distancia_cm\":%s,"
          "\"aceleracao_valida\":%s,"
          "\"ax_ms2\":%s,"
          "\"ay_ms2\":%s,"
          "\"az_ms2\":%s,"
          "\"aceleracao_total_ms2\":%s,"
          "\"aceleracao_desvio_ms2\":%s,"
          "\"aceleracao_desvio_pico_ms2\":%s,"
          "\"amostras_aceleracao\":%lu"
        "}",
        GROUP_ID,
        DEVICE_ID,
        static_cast<unsigned long>(now),
        static_cast<unsigned long>(sequenceNumber),
        static_cast<unsigned long>(now - windowStartedAt),
        temperatureValid ? "true" : "false",
        tempText,
        humidityText,
        distanceValid ? "true" : "false",
        distanceText,
        accelerationValid ? "true" : "false",
        axText,
        ayText,
        azText,
        magnitudeText,
        deviationText,
        peakText,
        static_cast<unsigned long>(accelerationSamples)
    );

    if (
        length < 0 ||
        static_cast<size_t>(length) >= sizeof(payload)
    ) {
        Serial.println("[JSON] Buffer insuficiente.");
    } else {
        Serial.println(payload);

        if (mqttClient.connected()) {
            const bool sent = mqttClient.publish(
                MQTT_TOPIC,
                payload,
                false
            );

            Serial.println(
                sent
                    ? "[MQTT] Publicacao enviada."
                    : "[MQTT] Falha na publicacao."
            );
        } else {
            Serial.println("[MQTT] Sem conexao.");
        }
    }

    // Esta versão não armazena janelas para reenvio offline.
    accelerationPeak = 0.0f;
    accelerationSamples = 0;
    windowStartedAt = now;
}

void setup() {
    Serial.begin(115200);
    delay(300);

    Serial.println();
    Serial.println("==================================");
    Serial.println("CP3 - Monitoramento multissensor");
    Serial.println("DHT22 + HC-SR04 + MPU6050");
    Serial.println("==================================");

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    digitalWrite(TRIG_PIN, LOW);

    dht.setup(DHT_PIN, DHTesp::DHT22);

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    mpuReady = mpu.begin(0x68, &Wire);

    if (mpuReady) {
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

        Serial.println("[MPU6050] Inicializado.");
    } else {
        Serial.println(
            "[MPU6050] Nao encontrado. Confira I2C e alimentacao."
        );
    }

    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);

    mqttClient.setServer(MQTT_HOST, MQTT_PORT);
    mqttClient.setBufferSize(1536);
    mqttClient.setKeepAlive(30);
    mqttClient.setSocketTimeout(1);

    char suffix[32];

    snprintf(
        suffix,
        sizeof(suffix),
        "%08lx-%08lx",
        static_cast<unsigned long>(
            ESP.getEfuseMac() & 0xFFFFFFFFULL
        ),
        static_cast<unsigned long>(esp_random())
    );

    mqttClientId = String(DEVICE_ID) + "-" + suffix;

    windowStartedAt = millis();

    startWifi();
}

void loop() {
    readAcceleration();
    readDistance();
    readTemperature();

    maintainConnections();
    publishTelemetry();

    delay(1);
}