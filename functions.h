#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include "header.h"
#include "pinconfiguration.h"

#define DHT_TYPE DHT22

// Function declarations
void setupDHT22();        // Setup DHT22 sensor
void dataPrinting();     // Read temperature and humidity from DHT22 sensor
void printDHT22Data();    // Print the data to the Serial Monitor
//String dataToWrite();
void handleMQTT();

// Create the DHT22 object
DHT dht(DHT_PIN, DHT_TYPE);

// Initialize global variables
float temperature = 0.0;
float humidity = 0.0;
 String logEntry = "";
// unsigned long lastTime = 0;

// Setup the DHT22 sensor
void setupDHT22() {
  dht.begin();  // Start the DHT sensor
  Serial.println("DHT22 Sensor Initialized");
}

// Read temperature and humidity from the DHT22 sensor
void readDHT22Data() {
  humidity = dht.readHumidity();        // Read humidity
  temperature = dht.readTemperature();  // Read temperature (in Celsius)

  // Check if readings failed
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Failed to read from DHT sensor!");
  
  }
  
}

// Print the DHT22 data to the Serial Monitor
// void printDHT22Data() {
//   Serial.print("Temperature: ");
//   Serial.print(temperature);
//   Serial.print("°C  Humidity: ");
//   Serial.print(humidity);
//   Serial.println("%");
// }



// Global variables
volatile int doorCount = 0;   // Counter for door open/close events
volatile bool doorState = false;  // Current state of the door (open/close)
unsigned long lastDebounceTime = 0;
float tempDS18B20 = 0.0;

// Create OneWire and DallasTemperature instances
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

PZEM004Tv30 pzem(PZEM_SERIAL, PZEM_RX_PIN, PZEM_TX_PIN);

void setupIRSensor();        // Setup IR sensor and interrupt
void calculateRPM();         // Calculate RPM based on pulse count

// Setup IR sensor and interrupt
void setupIRSensor() {
  pinMode(IR_SENSOR_PIN, INPUT);  // Set IR sensor pin as input
  attachInterrupt(digitalPinToInterrupt(IR_SENSOR_PIN), pulseDetected, RISING); // Interrupt on rising edge
}


// Calculate RPM based on pulse count
void calculateRPM() {
  if (millis() - lastTime >= 1000) {   // Calculate RPM every 1 second
    rpm = pulseCount * 60.0;            // Calculate RPM (pulses per second * 60)
    pulseCount = 0;                    // Reset pulse count

    Serial.print("Fan RPM: ");
    Serial.println(rpm);

    lastTime = millis();               // Reset last time for next RPM calculation
  }
}



// External Declaration
extern PubSubClient client;  // Declare 'client' as external
extern SemaphoreHandle_t sdMutex;  // Declare 'sdMutex' as external

// Function Declarations
void setupWiFi();
void setupMQTT();
void connectMQTT();
void reconnectMQTT();
void logDataToSD(const String &data);
void sendDataToCloud(void *parameter);
void sendGetRequest();
void mqttCallback(char* topic, byte* payload, unsigned int length);



// WiFi Setup
void setupWiFi() {
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\n\u2705 WiFi Connected!");
}


// Connect to MQTT Broker
void connectMQTT() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect("ESP32Client")) {
      Serial.println("Connected!");
      client.subscribe(mqttTopic);  // Subscribe to topic
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      delay(5000);
    }
  }
}

void sendGetRequest() {
    Serial.println("MQTT GET request received!");
    // Your MQTT request handling code goes here
}

// MQTT Callback function
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println("Message received: " + message);

  // If the message is "GET", trigger a GET request
  if (message == "GET") {
    sendGetRequest();
  }
}

// Send GET request to the server
void requestDataFromMQTT() {
    if (client.connected()) {
        String requestPayload = "{\"request\":\"data\"}";  // Example request JSON
        client.publish("device/request", requestPayload.c_str());
        Serial.println("📤 Sent MQTT request for data.");
    } else {
        Serial.println("❌ MQTT Disconnected! Attempting reconnection...");
        connectMQTT();
    }
}





// MQTT Reconnect
void reconnectMQTT() {
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (client.connect("ESP32Client")) {
            Serial.println("\u2705 Connected to MQTT!");
        } else {
            Serial.print("\u274C Failed, rc="); 
            Serial.print(client.state());
            Serial.println(" Retrying in 5 sec...");
            delay(5000);
        }
    }
}


// Setup MQTT
void setupMQTT() {
  client.setServer(mqttServer, mqttPort);
  client.setCallback(mqttCallback); // Set the MQTT callback function
}


void handleMQTT() {
    if (!client.connected()) {
        reconnectMQTT();  // Function to reconnect
    }
    client.loop();  // Process incoming messages
}


// SD Card Logging
void logDataToSD(const String &data) {
    if (xSemaphoreTake(sdMutex, portMAX_DELAY)) {  // Lock SD card access
        File file = SD.open("/log.txt", FILE_APPEND);
        if (file) {
            file.println(data);
            file.close();
            Serial.println("\u2705 Logged: " + data);
        } else {
            Serial.println("\u274C Failed to write to SD!");
        }
        xSemaphoreGive(sdMutex);
    }
}

// MQTT Sending Task
void sendDataToCloud(void *parameter) {
    while (1) {
        reconnectMQTT();
        if (xSemaphoreTake(sdMutex, portMAX_DELAY)) {  // Lock SD card access
            File file = SD.open("/log.txt", FILE_READ);
            if (!file) {
                Serial.println("\u274C Failed to open log file!");
            } else {
                while (file.available()) {
                    String line = file.readStringUntil('\n');
                    client.publish("iot/dataCM", line.c_str());
                    delay(500);
                }
                file.close();
                SD.remove("/log.txt");
                Serial.println("\u2705 Data sent & log cleared!");
            }
            xSemaphoreGive(sdMutex);
        }
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}










void IRAM_ATTR handleDoorInterrupt() {
    detachInterrupt(digitalPinToInterrupt(REED_SWITCH_PIN));  // Disable interrupt temporarily
    lastDebounceTime = millis();  // Save debounce start time
}

// Function to initialize the reed switch sensor
void initReedSwitch() {
    Serial.begin(115200);
    pinMode(REED_SWITCH_PIN, INPUT_PULLUP);

    // Reading sensor inside setup code for initial door status
    doorState = digitalRead(REED_SWITCH_PIN);
    Serial.println(doorState ? "OPEN" : "CLOSED");

    // Attach an interrupt to the reed switch pin
    attachInterrupt(digitalPinToInterrupt(REED_SWITCH_PIN), handleDoorInterrupt, CHANGE);
}

// Function to process door state changes
void processDoorState() {
    unsigned long currentTime = millis();
    if ((currentTime - lastDebounceTime) > DEBOUNCE_TIME && lastDebounceTime != 0) {
        bool newState = digitalRead(REED_SWITCH_PIN);  // Read stable state

        // Only register change if the state is actually different
        if (newState != doorState) {
            doorState = newState;
            doorCount++;
            Serial.print("Door state changed: ");
            Serial.println(doorState ? "OPEN" : "CLOSED");
            Serial.print("Door open/close count: ");
            Serial.println(doorCount);
        }

        lastDebounceTime = 0;  // Reset debounce timer
        attachInterrupt(digitalPinToInterrupt(REED_SWITCH_PIN), handleDoorInterrupt, CHANGE);  // Re-enable interrupt
    }
}





// Function to initialize the temperature sensor
void initTemperatureSensor() {
    sensors.begin();  // Initialize the DS18B20 sensor
}


void initSerial() {
    CONSOLE_SERIAL.begin(115200);
}




// Function to get and print the temperature
void readTemperature() {
    sensors.requestTemperatures();  // Request temperature data
   tempDS18B20 = sensors.getTempCByIndex(0);  // Read temperature

    Serial.print("DS18B20 -> Temperature: ");
    Serial.println(tempDS18B20);  // Print temperature

   // delay(2000);  // Wait 2 seconds before next reading
}

// Function to read and print PZEM sensor data
void readPZEMData() {
    // Print the custom address of the PZEM
    CONSOLE_SERIAL.print("Custom Address:");
    CONSOLE_SERIAL.println(pzem.readAddress(), HEX);

    // Read sensor data
    float voltage = pzem.voltage();
    float current = pzem.current();
    float power = pzem.power();
    float energy = pzem.energy();

    // Check for valid data
    if (isnan(voltage)) {
        CONSOLE_SERIAL.println("Error reading voltage");
    } else if (isnan(current)) {
        CONSOLE_SERIAL.println("Error reading current");
    } else if (isnan(power)) {
        CONSOLE_SERIAL.println("Error reading power");
    } else if (isnan(energy)) {
        CONSOLE_SERIAL.println("Error reading energy");
    } else {
        // Print values
        CONSOLE_SERIAL.print("Voltage: ");  CONSOLE_SERIAL.print(voltage);  CONSOLE_SERIAL.println("V");
        CONSOLE_SERIAL.print("Current: ");  CONSOLE_SERIAL.print(current);  CONSOLE_SERIAL.println("A");
        CONSOLE_SERIAL.print("Power: ");    CONSOLE_SERIAL.print(power);    CONSOLE_SERIAL.println("W");
        CONSOLE_SERIAL.print("Energy: ");   CONSOLE_SERIAL.print(energy, 3);CONSOLE_SERIAL.println("kWh");
    }
    CONSOLE_SERIAL.println();
    delay(2000);
}

 String dataToWrite(float temperature, float humidity, float tempDS18B20, bool doorState, int doorCount){
   
   
        logEntry = "DHT Temp: " + String(temperature) + "°C, ";
    logEntry += "DHT Humidity: " + String(humidity) + "%, ";
    logEntry += "DS18B20 Temp: " + String(tempDS18B20) + "°C, ";
   logEntry += "Door Status: " + String(doorState) + ",";
    logEntry += "Door Count: " + String(doorCount) + "°C";

    return logEntry;
 }

 

void dataPrinting()
{
    Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print("°C  Humidity: ");
  Serial.print(humidity);
  Serial.println("%");
  Serial.print("DS18B20 Temperature: ");
  Serial.print(tempDS18B20);  // Print temperature
  Serial.println("%");
  Serial.print("Door Status: ");
  Serial.println(doorState ? "OPEN" : "CLOSED");
  Serial.print("Door open/close count: ");
  Serial.println(doorCount);

}


#endif
