#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include "header.h"
#include "pinconfiguration.h"

#define DHT_TYPE DHT11

// -----------------------------------------------------------------------------Function declarations--------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void setupDHT22();        // Setup DHT22 sensor
void dataPrinting();     // Read temperature and humidity from DHT22 sensor
void readDHT22Data();    // Print the data to the Serial 
void initrelay();
void getTimeStamp();
void rtcSetup();
void syncRTCWithNTP();
void setupIRSensor();        // Setup IR sensor and interrupt
void calculateRPM();         // Calculate RPM based on pulse count
void handleMQTT();
void relayoff();
void relayon();
void setupWiFi();
void setupMQTT();
void handshake();
void connectMQTT();
void reconnectMQTT();
void logDataToSD(const String &data);
void sendDataToCloud(void *parameter);
void sendGetRequest();
void mqttCallback(char* topic, byte* payload, unsigned int length);



bool wifiConnected = false;
bool mqttConnected = false;
unsigned long lastWiFiCheckTime = 0;
unsigned long lastMqttReconnectAttempt = 0;

String dataToPacket();
bool HSA_Flag = true;
String relayState = "";
String c = "";
String dateTimeStr = "";
String machineStatus = "high";

//String dataToWrite();

// Initialize global variables
float temperature = 0.0;
float humidity = 0.0;
float voltage=0.0;
float current=0.0;
float power=0.0;
float energy=0.0;
float frequency=0.0;
float pf=0.0;
const int blades = 5;
 String logEntry = "";
// unsigned long lastTime = 0;
bool flag = false;

// Enhanced sensor reading functions with error recovery
// Add these to your functions.h file, replacing the existing implementations

// Variables to store last valid readings
float lastValidTemperature = 0.0;
float lastValidHumidity = 0.0;
float lastValidDS18B20 = 0.0;
float lastValidVoltage = 0.0;
float lastValidCurrent = 0.0;
float lastValidPower = 0.0;
float lastValidEnergy = 0.0;
int dhtErrorCount = 0;
int ds18b20ErrorCount = 0;
int pzemErrorCount = 0;


// Global variables
volatile int doorCount = 0;   // Counter for door open/close events
volatile bool doorState = true;  // Current state of the door (open/close)
unsigned long lastDebounceTime = 0;
float tempDS18B20 = 0.0;


//-----------------------------------------------------------------------------Object Declrations----------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// Create the DHT22 object
DHT dht(DHT_PIN, DHT_TYPE);

// Create OneWire and DallasTemperature instances
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

//Energy-Meter object
PZEM004Tv30 pzem(Serial2, PZEM_RX_PIN, PZEM_TX_PIN);

// Change to RTC_DS1307 rtc; if using DS1307
RTC_DS3231 rtc;  
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "in.pool.ntp.org", 19800, 60000);


// -------------------------------------------------------------------------External Declaration-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------

extern PubSubClient client;  // Declare 'client' as external
extern SemaphoreHandle_t sdMutex;  // Declare 'sdMutex' as external

//--------------------------------------------------------------------------------Relay --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


void initrelay(){
  pinMode(Relay,OUTPUT);
      digitalWrite(Relay,HIGH);
      relayState = "HIGH";
}


void relayoff(){
      digitalWrite(Relay,LOW);
      relayState = "LOW";
}

void relayon(){
      digitalWrite(Relay,HIGH);
      relayState = "HIGH";
}


// ---------------------------------------------------------------------------- DHT22 sensor --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void setupDHT22() {
  dht.begin();  // Start the DHT sensor
  Serial.println("DHT22 Sensor Initialized");
   }

// Read DHT22 data with error handling
void readDHT22Data() {
    float newHumidity = dht.readHumidity();
    float newTemperature = dht.readTemperature();
    
    if (isnan(newTemperature) || isnan(newHumidity)) {
        dhtErrorCount++;
        Serial.println("⚠️ Failed to read from DHT sensor! Error count: " + String(dhtErrorCount));
        
        // After 3 consecutive errors, use last valid readings if available
        if (dhtErrorCount >= 3 && lastValidTemperature != 0.0) {
            temperature = lastValidTemperature;
            humidity = lastValidHumidity;
            Serial.println("Using last valid DHT readings: Temp=" + String(temperature) + 
                           "°C, Humidity=" + String(humidity) + "%");
        }
    } else {
        // Good reading - update values and reset error count
        temperature = newTemperature;
        humidity = newHumidity;
        lastValidTemperature = temperature;
        lastValidHumidity = humidity;
        dhtErrorCount = 0;
    }
}


//------------------------------------------------------------------------DS18B20(Internal Temp.)--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------



// Function to initialize the temperature sensor
void initTemperatureSensor() {
    sensors.begin();  // Initialize the DS18B20 sensor
  }

// Read DS18B20 temperature with error handling
void readTemperature() {
    sensors.requestTemperatures();
    float reading = sensors.getTempCByIndex(0);
    
    // Check for error readings
    if (reading == DEVICE_DISCONNECTED_C || reading == -127.0) {
        ds18b20ErrorCount++;
        Serial.println("⚠️ Failed to read from DS18B20 sensor! Error count: " + String(ds18b20ErrorCount));
        
        // After 3 consecutive errors, use last valid reading if available
        if (ds18b20ErrorCount >= 3 && lastValidDS18B20 != 0.0) {
            tempDS18B20 = lastValidDS18B20;
            Serial.println("Using last valid DS18B20 reading: " + String(tempDS18B20) + "°C");
        }
    } else {
        // Good reading - update value and reset error count
        tempDS18B20 = reading;
        lastValidDS18B20 = tempDS18B20;
        ds18b20ErrorCount = 0;
    }
}



//------------------------------------------------------------------------------FAN-RPM--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


// Setup IR sensor and interrupt
void setupIRSensor() {
  pinMode(IR_SENSOR_PIN, INPUT_PULLUP);  // Set IR sensor pin as input
  attachInterrupt(digitalPinToInterrupt(IR_SENSOR_PIN), pulseDetected, RISING); // Interrupt on rising edge
}


// Calculate RPM based on pulse count
void calculateRPM() {
  if (millis() - lastTime >= 1000) {   // Calculate RPM every 1 second
    rpm = (pulseCount / (float)blades) * 60.0;            // Calculate RPM (pulses per second * 60)
    pulseCount = 0;                    // Reset pulse count

    Serial.print("Fan RPM: ");
    Serial.println(rpm);

    lastTime = millis();               // Reset last time for next RPM calculation
  }
}


// ------------------------------------------------------------------------------ Door Status Handling ----------------------------------------------------------------------------------------------------------------------------------------------------------------------


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
    Serial.println(doorState ? "CLOSED" : "OPEN");

    // Attach an interrupt to the reed switch pin
    attachInterrupt(digitalPinToInterrupt(REED_SWITCH_PIN), handleDoorInterrupt, CHANGE);
}

// Function to process door state changes
void processDoorState() {
    unsigned long currentTime = millis();
    if ((currentTime - lastDebounceTime) > DEBOUNCE_TIME && lastDebounceTime != 0) {
        bool newState = !digitalRead(REED_SWITCH_PIN);  // Read stable state

        // Only register change if the state is actually different
        if (newState != doorState) {
            doorState = newState;
            doorCount++;
      Serial.print("Door state changed: ");
    c = doorState ? "OPEN" : "CLOSED";
  Serial.println(c);
      Serial.print("Door open/close count: ");
      Serial.println(doorCount);
        }

        lastDebounceTime = 0;  // Reset debounce timer
        attachInterrupt(digitalPinToInterrupt(REED_SWITCH_PIN), handleDoorInterrupt, CHANGE);  // Re-enable interrupt
    }
}


// --------------------------------------------------------------------------- -Energy Meter Handling -------------------------------------------------------------------------------------------------------------------------------------------------------------------------



// Read PZEM data with error handling
void readPZEMData() {
    // Read all values
    float newVoltage = pzem.voltage();
    float newCurrent = pzem.current();
    float newPower = pzem.power();
    float newEnergy = pzem.energy();
    frequency = pzem.frequency();
    pf = pzem.pf();
    
    // Check if any critical readings are invalid
    bool hasError = isnan(newVoltage) || isnan(newCurrent) || 
                    isnan(newPower) || isnan(newEnergy);
    
    if (hasError) {
        pzemErrorCount++;
        Serial.println("⚠️ Error reading from PZEM sensor! Error count: " + String(pzemErrorCount));
        
        // After 3 consecutive errors, use last valid readings for invalid values
        if (pzemErrorCount >= 3) {
            if (isnan(newVoltage) && lastValidVoltage != 0.0) {
                voltage = lastValidVoltage;
                Serial.println("Using last valid voltage: " + String(voltage) + "V");
            } else if (!isnan(newVoltage)) {
                voltage = newVoltage;
                lastValidVoltage = voltage;
            }
            
            if (isnan(newCurrent) && lastValidCurrent != 0.0) {
                current = lastValidCurrent;
                Serial.println("Using last valid current: " + String(current) + "A");
            } else if (!isnan(newCurrent)) {
                current = newCurrent;
                lastValidCurrent = current;
            }
            
            if (isnan(newPower) && lastValidPower != 0.0) {
                power = lastValidPower;
                Serial.println("Using last valid power: " + String(power) + "W");
            } else if (!isnan(newPower)) {
                power = newPower;
                lastValidPower = power;
            }
            
            if (isnan(newEnergy) && lastValidEnergy != 0.0) {
                energy = lastValidEnergy;
                Serial.println("Using last valid energy: " + String(energy) + "kWh");
            } else if (!isnan(newEnergy)) {
                energy = newEnergy;
                lastValidEnergy = energy;
            }
        }
    } else {
        // All readings good - update values and reset error count
        voltage = newVoltage;
        current = newCurrent;
        power = newPower;
        energy = newEnergy;
        
        lastValidVoltage = voltage;
        lastValidCurrent = current;
        lastValidPower = power;
        lastValidEnergy = energy;

        
        pzemErrorCount = 0;
    }
}


//----------------------------------------------------------------------------------RTC Config--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void rtcSetup(){
  Wire.begin();
  // ✅ Start NTP Client
  timeClient.begin();

    // ✅ Initialize RTC
    if (!rtc.begin()) {
        Serial.println("❌ RTC Not Found!");
        while (1);
    }
     
    if (rtc.lostPower()) {
        Serial.println("⚠️ RTC lost power! Syncing time...");
        syncRTCWithNTP();
    }
    //syncRTCWithNTP();

}

void getTimeStamp(){

DateTime now = rtc.now();

      dateTimeStr = String(now.year()) + "-";
      dateTimeStr += String(now.month()) + "-";
      dateTimeStr += String(now.day()) + "T";
      dateTimeStr += String(now.hour()) + ":";
      dateTimeStr += String(now.minute()) + ":";
      dateTimeStr += String(now.second()) + "";
Serial.println(dateTimeStr);
}


// ✅ Sync RTC Time with NTP
void syncRTCWithNTP() {
    Serial.println("🌍 Syncing RTC with NTP...");
    timeClient.update();

    int ntpHour = timeClient.getHours();
    int ntpMinute = timeClient.getMinutes();
    int ntpSecond = timeClient.getSeconds();
    int ntpDay = timeClient.getDay();  // 0 = Sunday
    int ntpEpoch = timeClient.getEpochTime();

    DateTime ntpTime = DateTime(ntpEpoch);
    
    // ✅ Compare RTC time with NTP time
    DateTime now = rtc.now();
if (abs((int32_t)(now.unixtime() - ntpTime.unixtime())) > 30) {  
    rtc.adjust(ntpTime);
    Serial.println("✅ RTC Updated!");
}
 else {
        Serial.println("⏳ RTC Time is already accurate.");
    }
}



//----------------------------------------------------------------- WiFi Configuration --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


// WiFi Setup
void setupWiFi() {
    int retryCount = 0;
    unsigned long retryDelay = 500; // Start with 500ms
    const int MAX_RETRIES = 10;
    
    WiFi.disconnect();
    delay(100);
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    
    while (WiFi.status() != WL_CONNECTED && retryCount < MAX_RETRIES) {
        delay(retryDelay);
        Serial.print(".");
        retryCount++;
        
        // Reset WiFi connection if taking too long
        if (retryCount == 5) {
            Serial.println("\nResetting WiFi connection...");
            WiFi.disconnect();
            delay(1000);
            WiFi.begin(ssid, password);
        }
        
        // Increase delay up to 3 seconds
        if (retryDelay < 3000) {
            retryDelay = retryDelay * 1.5;
        }
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n✅ WiFi Connected!");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
        wifiConnected = true;
    } else {
        Serial.println("\n❌ WiFi Connection Failed!");
        wifiConnected = false;
    }
}


// Periodically check WiFi and reconnect if needed
void checkWiFiConnection() {
    const unsigned long CHECK_INTERVAL = 30000; // 30 seconds
    unsigned long currentTime = millis();
    
    if (currentTime - lastWiFiCheckTime >= CHECK_INTERVAL) {
        lastWiFiCheckTime = currentTime;
        
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("WiFi connection lost. Reconnecting...");
            wifiConnected = false;
            setupWiFi();
            
            if (wifiConnected) {
                connectMQTT();
            }
        }
    }
}


//-------------------------------------------------------------------- MQTT Connection -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// Setup MQTT
void setupMQTT() {
  client.setServer(mqttServer, mqttPort);
  client.setCallback(mqttCallback); // Set the MQTT callback function
}


void handleMQTT() {
    if (!client.connected()) {
        mqttConnected = false;
        reconnectMQTT();
    } else {
        client.loop();
    }
}


// MQTT Reconnect
void reconnectMQTT() {
    const unsigned long RECONNECT_INTERVAL = 5000; // 5 seconds
    unsigned long currentTime = millis();
    
    if (!client.connected() && currentTime - lastMqttReconnectAttempt >= RECONNECT_INTERVAL) {
        lastMqttReconnectAttempt = currentTime;
        
        // Check WiFi connection first
        if (WiFi.status() != WL_CONNECTED) {
            setupWiFi();
        }
        
        if (WiFi.status() == WL_CONNECTED) {
            Serial.print("Attempting MQTT reconnection...");
            
            // Create unique client ID
            String clientId = "ESP32Client-";
            clientId += String(millis() & 0xffff);
            
            if (client.connect(clientId.c_str(), mqttUser, mqttPassword)) {
                Serial.println("✅ Reconnected!");
                client.subscribe(HSTopic);
                mqttConnected = true;
                
                // Publish reconnection message
                client.publish("iot/status", (sensor_id + " reconnected").c_str());
            } else {
                Serial.print("Failed, rc=");
                Serial.println(client.state());
                mqttConnected = false;
            }
        }
    }
}



// Connect to MQTT Broker
void connectMQTT() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Cannot connect to MQTT - WiFi not connected!");
        mqttConnected = false;
        return;
    }
    
    int retryCount = 0;
    const int MAX_RETRIES = 5;
    
    while (!client.connected() && retryCount < MAX_RETRIES) {
        Serial.print("Connecting to MQTT...");
        
        // Create unique client ID using timestamp
        String clientId = "ESP32Client-";
        clientId += String(millis() & 0xffff); // Use last 16 bits of millis for variety
        
        if (client.connect(clientId.c_str(), mqttUser, mqttPassword)) {
            Serial.println("Connected!");
            client.subscribe(HSTopic);
            mqttConnected = true;
        } else {
            Serial.print("Failed, rc=");
            Serial.print(client.state());
            Serial.println(" Retrying...");
            
            delay((retryCount + 1) * 1000); // Increasing delay
            retryCount++;
        }
    }
    
    if (!client.connected()) {
        mqttConnected = false;
    }
}


// void handshake(){
//      while(!HSA_Flag) {
//         if (!client.connected()) {
//         connectMQTT();
//     }
//     client.loop();  
//     // Send -1 for handshaking    Serial.println("Sending handshake...");
//       bool success = client.publish("iot/handshake", "-1");
//       if(success)
//         Serial.println("Handshake message sended");
//       else
//         Serial.println("Can't Handshake message");
//         delay(2000);
// }
// }

// void sendGetRequest() {
//     Serial.println("MQTT GET request received!");
//     // Your MQTT request handling code goes here
// }

// MQTT Callback function
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println("Message received: " + message);

  // If the message is "GET", trigger a GET request
    if (message == "1") {
    HSA_Flag = true;
    Serial.println("Handshake successful, starting to send sensor data...");
  }
  else if (message == "on") {
    relayon();
    machineStatus = "high";
    Serial.println("RELAY ON");
  }
    else if (message == "off") {
    relayoff();
    machineStatus= "low";
    Serial.println("RELAY OFF");
  }
  
}

// // Send GET request to the server
// void requestDataFromMQTT() {
//     if (client.connected()) {
//         String requestPayload = "{\"request\":\"data\"}";  // Example request JSON
//         client.publish("device/request", requestPayload.c_str());
//         Serial.println("📤 Sent MQTT request for data.");
//     } else {
//         Serial.println("❌ MQTT Disconnected! Attempting reconnection...");
//         connectMQTT();
//     }
// }


//------------------------------------------------------------------------------- SD-Data Logging ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// Enhanced SD card logging with error recovery
void logDataToSD(const String &data) {
    static int sdErrorCount = 0;
    
    if (xSemaphoreTake(sdMutex, portMAX_DELAY)) {  // Lock SD card access
        File file = SD.open("/log.txt", FILE_APPEND);
        
        if (file) {
            size_t bytesWritten = file.println(data);
            file.close();
            
            if (bytesWritten > 0) {
                Serial.println("✅ Logged to SD card");
                sdErrorCount = 0;
                flag = false;
            } else {
                sdErrorCount++;
                Serial.println("⚠️ SD write error (0 bytes written). Error count: " + String(sdErrorCount));
                flag = true;
            }
        } else {
            sdErrorCount++;
            Serial.println("⚠️ Failed to open log file! Error count: " + String(sdErrorCount));
            flag = true;
            
            // Try to reinitialize SD card after multiple failures
            if (sdErrorCount >= 5) {
                Serial.println("Attempting to reinitialize SD card...");
                SD.end();
                delay(500);
                
                if (SD.begin(SD_CS)) {
                    Serial.println("✅ SD card reinitialized successfully");
                    sdErrorCount = 0;
                } else {
                    Serial.println("❌ SD card reinitialization failed");
                }
            }
        }
        
        xSemaphoreGive(sdMutex);
    }
}


//------------------------------------------------------------------------------------------ MQTT-Data Sending ---------------------------------------------------------------------------------------------------------------------------------------------------------------

// MQTT Sending Task
// void sendDataToCloud(void *parameter) {
//     while (1) {
//         reconnectMQTT();
//         if (xSemaphoreTake(sdMutex, portMAX_DELAY)) {  // Lock SD card access
//             File file = SD.open("/log.txt", FILE_READ);
//             if (!file) {
//                 Serial.println("\u274C Failed to open log file!");
//                 flag = true;
//             } else {
//                 while (file.available()) {
//                     String line = file.readStringUntil('\n');
//                     client.publish("iot/dataCM", line.c_str());
                  
//                     delay(500);
//                 }
//                 file.close();
//                 SD.remove("/log.txt");
//                 Serial.println("\u2705 Data sent & log cleared!");
//                 flag  = false;
//             }
//             xSemaphoreGive(sdMutex);
//         }
//         vTaskDelay(10000 / portTICK_PERIOD_MS);
    
//     }
// }


// Enhanced MQTT publishing task with better error handling
// Replace your existing sendDataToCloud function with this implementation

void sendDataToCloud(void *parameter) {
    static unsigned long lastPublishTime = 0;
    static int mqttPublishErrors = 0;
    const int MAX_PUBLISH_ERRORS = 5;
    
    while (1) {
        // First check connections
        bool nowConnected = (WiFi.status() == WL_CONNECTED && client.connected());
        
        if (!nowConnected) {
            // Only attempt reconnection if enough time has passed
            static unsigned long lastReconnectAttempt = 0;
            if (millis() - lastReconnectAttempt > 5000) { // Try every 5 seconds
                lastReconnectAttempt = millis();
                
                // First check WiFi, then MQTT
                if (WiFi.status() != WL_CONNECTED) {
                    Serial.println("WiFi disconnected, reconnecting...");
                    setupWiFi();
                }
                
                if (WiFi.status() == WL_CONNECTED && !client.connected()) {
                    Serial.println("MQTT disconnected, reconnecting...");
                    
                    String clientId = "ESP32Client-";
                    clientId += String(millis() & 0xffff);
                    
                    if (client.connect(clientId.c_str(), mqttUser, mqttPassword)) {
                        Serial.println("✅ MQTT reconnected in publish task");
                        client.subscribe(HSTopic);
                    }
                }
            }
        }
        
        // If connected and we have data to send
        if (client.connected() && logEntry.length() > 10) {
            // Only publish at proper intervals or if retrying after error
            if (millis() - lastPublishTime >= 1000 || mqttPublishErrors > 0) {
                lastPublishTime = millis();
                
                bool publishSuccess = client.publish("iot/dataCM", logEntry.c_str());
                
                if (publishSuccess) {
                    Serial.println("✅ Data published to MQTT");
                    mqttPublishErrors = 0;
                } else {
                    mqttPublishErrors++;
                    Serial.print("⚠️ MQTT publish failed. Error count: ");
                    Serial.println(mqttPublishErrors);
                    
                    // If too many errors, force MQTT reconnection
                    if (mqttPublishErrors >= MAX_PUBLISH_ERRORS) {
                        Serial.println("Too many publish errors. Forcing reconnection...");
                        client.disconnect();
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                    }
                }
            }
        }
        
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}


// ---------------------------------------------------------------------------------- Data Packaging -----------------------------------------------------------------------------------------------------------------------------------------------------------------------


String dataToPacket(float temperature, float humidity, float tempDS18B20, bool doorState, 
                   int doorCount, float rpm, float voltage, float current, float power, 
                   float energy, String machineStatus, String dateTimeStr) {
    // Create a buffer large enough for all data
    char buffer[512];
    
    // Format the entire string at once using sprintf
    snprintf(buffer, sizeof(buffer),
        "Sensor ID: %s, DHT Temp: %.1fC, DHT Humidity: %.1f, DS18B20 Temp: %.1fC, "
        "Door Status: %d, Door Count: %d, Voltage: %.1fV, Current: %.3fA, Power: %.1fW, "
        "Energy: %.3fKWH, power_status: %s, Fan RPM: %.1f, time: %s",
        sensor_id.c_str(), temperature, humidity, tempDS18B20, 
        doorState, doorCount, voltage, current, power,
        energy, machineStatus.c_str(), rpm, dateTimeStr.c_str());
    
    // Convert to Arduino String
    logEntry = String(buffer);
    
    Serial.println("Log Entry Completed!");
    Serial.println(logEntry);
    
    return logEntry;
}

 
//------------------------------------------------------------------------------------ Data Printing ----------------------------------------------------------------------------------------------------------------------------------------------------------------------


void dataPrinting()
{
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print("°C  Humidity: ");
  Serial.print(humidity);
  Serial.println("%");
  Serial.print("DS18B20 Temperature: ");
  Serial.print(tempDS18B20);  // Print temperature
  Serial.println("°C");
  Serial.print("Door Status: ");
  Serial.println(doorState ? "CLOSED" : "OPEN");
  Serial.print("Door open/close count: ");
  Serial.println(doorCount);
  Serial.print("Fan RPM: ");
  Serial.println(rpm); 
  Serial.print("Power Status: ");
  Serial.println(relayState);  
  Serial.print("Voltage: ");  Serial.print(voltage);  Serial.println("V");
  Serial.print("Current: ");  Serial.print(current);  Serial.println("A");
  Serial.print("Power: ");    Serial.print(power);    Serial.println("W");
  Serial.print("Energy: ");   Serial.print(energy);   Serial.println("kWh");
  Serial.print("frequency: ");   Serial.print(frequency);Serial.println("kWh");
  Serial.print("pf : ");   Serial.println(pf);
    
}


//-----------------------------------------------------------------------------------------------Watchdog-------------------------------------------

void watchdogTask(void *pvParameters) {
    esp_task_wdt_add(NULL);  // Subscribe this task to the TWDT
    while (1) {
        esp_task_wdt_reset();  // Feed the watchdog
        // Optional: Add monitoring code here
        vTaskDelay(10000 / portTICK_PERIOD_MS);  // Feed every 10 seconds
    }
}





#endif
