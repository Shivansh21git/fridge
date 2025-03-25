#ifndef PINCONFIGURATION_H
#define PINCONFIGURATION_H

#define ONE_WIRE_BUS 33  // Define the GPIO pin for DS18B20 sensor

// Define the pin for the IR sensor
#define IR_SENSOR_PIN 32

// Definr Relay Pin
#define Relay 4

// Define the pin for the DHT22 sensor
#define DHT_PIN 27  // Change this to the correct pin connected to DHT22 sensor

// PZEM004Tv30 pzem(PZEM_SERIAL, PZEM_RX_PIN, PZEM_TX_PIN);
#define PZEM_RX_PIN 16
#define PZEM_TX_PIN 17

#define REED_SWITCH_PIN 14  // GPIO pin for the reed switch
#define DEBOUNCE_TIME 50    // Debounce time in milliseconds

#define SD_CS 5  // SD Card Chip Select

// WiFi & MQTT Credentials
// const char* ssid = "Cool Max admin";
// const char* password = "admin@123";
const char* ssid = "Kumar 2";
const char* password = "ipl@2023";
// const char* ssid = "shiv5G";
// const char* password = "123456789";
// const char* mqtt_server = "your_MQTT_BROKER";
const int mqttPort = 1883;
const char* HSTopic = "Ref818/in";
String sensor_id = "Ref818";

// const char* HSTopic = "topic/Handshake";
const char* mqttServer = "api.coolmaxcloud.com";  // Replace with your broker
const char* mqttUser = "user";  // Replace with your broker
const char* mqttPassword = "password";  // Replace with your broker




#endif
