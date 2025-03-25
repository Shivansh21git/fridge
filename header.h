#ifndef HEADER_H
#define HEADER_H

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>  // Include DHT sensor library
#include <RTClib.h>          // Library for DS3231/DS1307 RTC
#include <NTPClient.h>       // For getting time from NTP server
#include <WiFi.h>
#include <WiFiUdp.h>         // UDP for NTP
#include <Wire.h>
#include <esp_task_wdt.h>
#include <PZEM004Tv30.h>
#include <PubSubClient.h>
#include <SD.h>
#include <SPI.h>
// #include <Arduino.h>

extern volatile int pulseCount;   // Variable to count pulses
extern float rpm;                 // Variable to store calculated RPM
extern unsigned long lastTime;    // To track time for RPM calculation

void IRAM_ATTR pulseDetected();


extern DHT dht;           // Declare the DHT sensor object
extern float temperature; // Variable to store temperature
extern float humidity;    // Variable to store humidity
extern unsigned long lastTime;  // For managing DHT22 read intervals



void readDHT22Data();     // Function to read data from the DHT22 sensor
void printDHT22Data();    // Function to print the DHT22 data

extern const int mqttPort;
extern const char* mqttTopic;



#endif
