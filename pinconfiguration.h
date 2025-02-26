#ifndef PINCONFIGURATION_H
#define PINCONFIGURATION_H

#define ONE_WIRE_BUS 33  // Define the GPIO pin for DS18B20 sensor

// Define the pin for the IR sensor
#define IR_SENSOR_PIN 32
// Definr Relay Pin
#define Relay 4
// Define the pin for the DHT22 sensor
#define DHT_PIN 27  // Change this to the correct pin connected to DHT22 sensor

#define PZEM_RX_PIN 16
#define PZEM_TX_PIN 17


// PZEM004Tv30 pzem(PZEM_SERIAL, PZEM_RX_PIN, PZEM_TX_PIN);

#define REED_SWITCH_PIN 14  // GPIO pin for the reed switch
#define DEBOUNCE_TIME 50    // Debounce time in milliseconds

#define SD_CS 5  // SD Card Chip Select

// WiFi & MQTT Credentials
const char* ssid = "Kumar 2";
const char* password = "ipl@2023";
// const char* ssid = "Airtel_amit_4422";
// const char* password = "Abhay4422";
// const char* mqtt_server = "your_MQTT_BROKER";
const int mqttPort = 1883;
const char* HSTopic = "sensor_data/test";
const char* RQTopic = "Ref01/read";
// const char* HSTopic = "topic/Handshake";
const char* mqttServer = "api.coolmaxcloud.com";  // Replace with your broker
const char* mqttUser = "user";  // Replace with your broker
const char* mqttPassword = "password";  // Replace with your broker

//const int mqttPort = 1883;

#define MQTT_PUBLISH_TOPIC "device/data"
#define MQTT_SUBSCRIBE_TOPIC "device/control"



#endif
