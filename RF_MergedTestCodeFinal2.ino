#include "header.h"
#include "pinconfiguration.h"
#include "functions.h"
float temp;
float  hum;
float  inTemp;


WiFiClient espClient;
PubSubClient client(espClient);
SemaphoreHandle_t sdMutex;

volatile int pulseCount = 0;  // Initialize pulse count
float rpm = 0;                // Initialize RPM
unsigned long lastTime = 0;   // Initialize last time for RPM calculation

// Interrupt function to detect IR pulses
void IRAM_ATTR pulseDetected() {
  pulseCount++;  // Increment pulse count whenever IR sensor detects a fan blade
}



void setup() {
   initrelay();
  Serial.begin(115200);  // Initialize Serial Monitor
  setupWiFi();
  //Setup MQTT
setupMQTT();

  // Connect to MQTT Broker
 connectMQTT();
 
  initTemperatureSensor();  // Initialize the temperature sensor


  initReedSwitch();  // REED Switch

  setupDHT22();  // Initialize the DHT22 sensor

  setupIRSensor();      // Setup the IR sensor and interrupt
  rtcSetup();
  syncRTCWithNTP();

  lastTime = millis();  // Initialize last time
 // setupWiFi();
 client.setServer(mqttServer, 1883);

  if (!SD.begin(SD_CS)) {
    Serial.println("\u274C SD Card Mount Failed!");
    return;
  }

  sdMutex = xSemaphoreCreateMutex();  // Initialize Mutex for SD Card
 reconnectMQTT();
  // handshake();
  
  //client.publish("iot/handshake", "-1");

  // Create FreeRTOS Task
 xTaskCreatePinnedToCore(sendDataToCloud, "MqttTask", 4096, NULL, 1, NULL, 0);
}

void loop() {

  client.loop();
 handleMQTT();  // Maintain MQTT connection

    // Request data every 10 seconds
    // static unsigned long lastRequestTime = 0;
    // if (millis() - lastRequestTime >= 10000) {
    //     lastRequestTime = millis();
    //     requestDataFromMQTT();
    // }

  // Read sensors data every 2 seconds
  if (millis() - lastTime >= 2000) {  // Read every 2 seconds
    readDHT22Data();                  // Read data from DHT22 sensor
    readTemperature();                // Ds18b20 Temperature
    readPZEMData();  // Electrical
    calculateRPM();  // Continuously calculate RPM every second
    getTimeStamp();
    dataToPacket(temperature,humidity,tempDS18B20,doorState,doorCount,voltage,current,power,energy,frequency,pf,rpm,relayState,dateTimeStr);
    logDataToSD(logEntry); 
     dataPrinting();
    lastTime = millis();              // Reset the timer
  }
    // 🔄 Check time every 10 minutes (600000 ms)
  static unsigned long lastSync = 0;
  if (millis() - lastSync > 120000) {
      lastSync = millis();
      syncRTCWithNTP();
  }


  processDoorState();  // Handle door open/close events
 
  //------------------Printing Data TO Serial Moniter-----------------------------------------------------------




  delay(5000);

    //logDataToSD(data);
}


