#ifndef PINCONFIGURATION_H
#define PINCONFIGURATION_H

#define ONE_WIRE_BUS 4  // Define the GPIO pin for DS18B20 sensor

// Define the pin for the IR sensor
#define IR_SENSOR_PIN 32

// Define the pin for the DHT22 sensor
#define DHT_PIN 4  // Change this to the correct pin connected to DHT22 sensor

#define PZEM_RX_PIN 16
#define PZEM_TX_PIN 17

#define PZEM_SERIAL Serial2
#define CONSOLE_SERIAL Serial
// PZEM004Tv30 pzem(PZEM_SERIAL, PZEM_RX_PIN, PZEM_TX_PIN);

#define REED_SWITCH_PIN 27  // GPIO pin for the reed switch
#define DEBOUNCE_TIME 50    // Debounce time in milliseconds

#define SD_CS 5  // SD Card Chip Select

// WiFi & MQTT Credentials
const char* ssid = "shiv5G";
const char* password = "123456789";
// const char* mqtt_server = "your_MQTT_BROKER";
const int mqttPort = 1883;
const char* mqttTopic = "your/topic";
const char* mqttServer = "broker.hivemq.com";  // Replace with your broker
//const int mqttPort = 1883;

#define MQTT_PUBLISH_TOPIC "device/data"
#define MQTT_SUBSCRIBE_TOPIC "device/control"



#endif
