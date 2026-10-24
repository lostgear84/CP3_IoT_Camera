#pragma once

#include <Arduino.h>

// Identificação
constexpr char GROUP_ID[] = "grupo01";
constexpr char DEVICE_ID[] = "esp32-grupo01-01";

// Wi-Fi do Wokwi
constexpr char WIFI_SSID[] = "Wokwi-GUEST";
constexpr char WIFI_PASSWORD[] = "";

// Broker de demonstração
constexpr char MQTT_HOST[] = "broker.hivemq.com";
constexpr uint16_t MQTT_PORT = 1883;
constexpr char MQTT_TOPIC[] = "fiap/CP3/grupo01/";

// DHT22
constexpr uint8_t DHT_PIN = 15;

// HC-SR04
constexpr uint8_t TRIG_PIN = 18;
constexpr uint8_t ECHO_PIN = 19;

// MPU6050 - I2C
constexpr uint8_t I2C_SDA_PIN = 21;
constexpr uint8_t I2C_SCL_PIN = 22;

// Intervalos
constexpr uint32_t DHT_INTERVAL_MS = 2500;
constexpr uint32_t DISTANCE_INTERVAL_MS = 150;
constexpr uint32_t ACCEL_INTERVAL_MS = 20;
constexpr uint32_t PUBLISH_INTERVAL_MS = 1000;

constexpr uint32_t WIFI_RETRY_MS = 10000;
constexpr uint32_t MQTT_RETRY_MS = 5000;

// Timeout da leitura ultrassônica
constexpr uint32_t ECHO_TIMEOUT_US = 25000;

// Referência física para cálculo da métrica de movimento
constexpr float GRAVITY_MS2 = 9.80665f;