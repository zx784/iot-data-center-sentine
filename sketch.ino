/*
 * DATA CENTER SENTINEL - Advanced IoT Monitoring System
 * Combines Environmental Control (Temperature Hysteresis) 
 * with Active Security (Arm/Disarm Intruder Detection)
 * 
 * Hardware Components:
 * - DHT22: Temperature/Humidity Sensor
 * - PIR: Motion Detector
 * - Reed Switch: Door Sensor
 * - Relay: Cooling Fan Control
 * - Buzzer: Alarm Siren
 * - Push Button: Arm/Disarm Toggle
 * - LED: System Status Indicator
 */

// Include necessary libraries for the project
#include <Arduino.h>        // Core Arduino framework for ESP32
#include <WiFi.h>          // WiFi connectivity library
#include <PubSubClient.h>  // MQTT protocol client library
#include <DHT.h>           // DHT temperature/humidity sensor library
#include <ArduinoJson.h>   // JSON serialization for MQTT messages

// ========================================
// CONFIGURATION - CHANGE THESE VALUES
// ========================================
const char* WIFI_SSID = "Wokwi-GUEST";      // WiFi network name (for Wokwi simulation)
const char* WIFI_PASSWORD = "";              // WiFi password (no password for Wokwi)

// MQTT Configuration (Using free public broker)
const char* MQTT_BROKER = "broker.hivemq.com";  // HiveMQ public MQTT broker address
const int MQTT_PORT = 1883;                      // Standard MQTT port (non-SSL)
const char* MQTT_CLIENT_ID = "ESP32_DataCenter_Sentinel";  // Unique identifier for this device
const char* MQTT_TOPIC_STATUS = "aiu/datacenter/status";   // Topic for publishing system status
const char* MQTT_TOPIC_COMMAND = "aiu/datacenter/command"; // Topic for receiving remote commands
const char* MQTT_TOPIC_ALARM = "aiu/datacenter/alarm";     // Topic for alarm notifications

// ========================================
// HARDWARE PIN DEFINITIONS
// ========================================
#define DHT_PIN 15              // GPIO pin for DHT22 Temperature/Humidity Sensor
#define PIR_PIN 13              // GPIO pin for PIR Motion Sensor
#define DOOR_PIN 12             // GPIO pin for Reed Switch (Door Sensor)
#define RELAY_FAN_PIN 26        // GPIO pin for Relay controlling cooling fan
#define BUZZER_PIN 4            // GPIO pin for Active Buzzer (Alarm Siren)
#define ARM_BUTTON_PIN 27       // GPIO pin for Push button to Arm/Disarm system
#define STATUS_LED_PIN 2        // GPIO pin for Status LED (ON = Armed)

#define DHT_TYPE DHT22          // Specify DHT sensor type (DHT22/AM2302)

// ========================================
// TEMPERATURE CONTROL THRESHOLDS
// ========================================
#define TEMP_HIGH_THRESHOLD 30.0    // Turn fan ON when temperature exceeds this value (°C)
#define TEMP_LOW_THRESHOLD 28.0     // Turn fan OFF when temperature drops below this (°C) - creates hysteresis
#define HUMIDITY_WARNING 70.0       // High humidity warning threshold (%)

// ========================================
// GLOBAL OBJECTS
// ========================================
DHT dhtSensor(DHT_PIN, DHT_TYPE);  // Create DHT sensor object with pin and type
WiFiClient wifiClient;              // Create WiFi client for network connection
PubSubClient mqttClient(wifiClient); // Create MQTT client using WiFi connection

// ========================================
// SYSTEM STATE VARIABLES
// ========================================
bool systemArmed = true;          // Security system armed status (true = armed, false = disarmed)
bool fanIsOn = false;             // Cooling fan state (true = running, false = stopped)
bool alarmActive = false;         // Alarm triggered state (true = alarm sounding)
unsigned long lastButtonPress = 0; // Timestamp of last button press (for debouncing)
unsigned long lastPublish = 0;     // Timestamp of last MQTT publish (for timing control)
unsigned long lastSensorRead = 0;  // Timestamp of last sensor read (for timing control)

// Sensor readings - store latest values from all sensors
float currentTemp = 0.0;           // Current temperature reading (°C)
float currentHumidity = 0.0;       // Current humidity reading (%)
bool motionDetected = false;       // PIR motion sensor state (true = motion detected)
bool doorOpen = false;             // Door sensor state (true = door open)
bool buttonPressed = false;        // Arm/Disarm button state (true = pressed)

// Status message - human-readable system status
String systemStatus = "INITIALIZING";  // Current system status text

// ========================================
// FUNCTION PROTOTYPES
// ========================================
void connectWiFi();                 // Establish WiFi connection
void connectMQTT();                 // Connect to MQTT broker
void readSensors();                 // Read all sensor values
void processEnvironmentalControl(); // Handle temperature-based fan control
void processSecuritySystem();       // Handle security monitoring and alarms
void handleButtonPress();           // Process arm/disarm button input
void soundAlarm(int duration);      // Trigger alarm buzzer
void publishStatus();               // Send system status via MQTT
void mqttCallback(char* topic, byte* payload, unsigned int length); // Handle incoming MQTT messages
void blinkLED(int times, int delayMs); // Blink status LED for visual feedback

// ========================================
// SETUP FUNCTION
// ========================================
void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);  // Set baud rate to 115200
  delay(1000);           // Wait for serial monitor to initialize
  
  // Print startup banner
  Serial.println("\n\n========================================");
  Serial.println("   DATA CENTER SENTINEL v2.0");
  Serial.println("   Advanced IoT Monitoring System");
  Serial.println("========================================\n");

  // Initialize GPIO pins as inputs or outputs
  pinMode(PIR_PIN, INPUT);                    // PIR sensor as input
  pinMode(DOOR_PIN, INPUT_PULLUP);            // Door sensor with internal pull-up resistor (HIGH = open)
  pinMode(ARM_BUTTON_PIN, INPUT_PULLUP);      // Button with internal pull-up (LOW = pressed)
  
  pinMode(RELAY_FAN_PIN, OUTPUT);             // Relay control as output
  pinMode(BUZZER_PIN, OUTPUT);                // Buzzer control as output
  pinMode(STATUS_LED_PIN, OUTPUT);            // LED indicator as output
  
  // Set initial output states - all OFF except LED based on armed status
  digitalWrite(RELAY_FAN_PIN, LOW);           // Fan OFF initially
  digitalWrite(BUZZER_PIN, LOW);              // Buzzer OFF initially
  digitalWrite(STATUS_LED_PIN, systemArmed ? HIGH : LOW);  // LED matches armed state

  // Initialize DHT sensor
  dhtSensor.begin();  // Start DHT22 sensor
  Serial.println("✓ Sensors initialized");

  // Connect to WiFi network
  connectWiFi();

  // Setup MQTT connection
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);  // Configure MQTT broker address and port
  mqttClient.setCallback(mqttCallback);           // Set callback function for incoming messages
  connectMQTT();                                  // Attempt initial connection

  // Startup sequence - provide user feedback
  Serial.println("\n✓ System Ready!");
  Serial.printf("✓ Security System: %s\n", systemArmed ? "ARMED" : "DISARMED");
  blinkLED(3, 200);  // Blink LED 3 times to indicate ready state
  
  delay(2000); // Allow sensors to stabilize before starting main loop
}

// ========================================
// MAIN LOOP
// ========================================
void loop() {
  // Maintain MQTT connection - reconnect if disconnected
  if (!mqttClient.connected()) {
    connectMQTT();  // Attempt to reconnect
  }
  mqttClient.loop();  // Process incoming MQTT messages and maintain connection

  // Read sensors every 2 seconds to avoid overwhelming the system
  if (millis() - lastSensorRead >= 2000) {  // Check if 2000ms have passed
    lastSensorRead = millis();  // Update timestamp
    readSensors();              // Read all sensor values
  }

  // Check for button press to arm/disarm system
  handleButtonPress();

  // Process environmental control logic (fan control based on temperature)
  processEnvironmentalControl();

  // Process security system logic (motion and door detection)
  processSecuritySystem();

  // Publish status to MQTT every 5 seconds for remote monitoring
  if (millis() - lastPublish >= 5000) {  // Check if 5000ms have passed
    lastPublish = millis();  // Update timestamp
    publishStatus();         // Send status update
  }

  // Update status LED to match armed state
  digitalWrite(STATUS_LED_PIN, systemArmed ? HIGH : LOW);

  delay(100); // Small delay for system stability and reduced CPU usage
}

// ========================================
// WiFi CONNECTION
// ========================================
void connectWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.mode(WIFI_STA);  // Set WiFi to station mode (client)
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);  // Start connection attempt

  int attempts = 0;  // Counter for connection attempts
  // Try to connect for up to 30 attempts (15 seconds)
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);        // Wait 500ms between attempts
    Serial.print("."); // Print progress indicator
    attempts++;        // Increment attempt counter
  }

  // Check connection result
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✓ WiFi Connected!");
    Serial.print("  IP Address: ");
    Serial.println(WiFi.localIP());  // Print assigned IP address
  } else {
    Serial.println("\n✗ WiFi Connection Failed!");
  }
}

// ========================================
// MQTT CONNECTION
// ========================================
void connectMQTT() {
  // Only attempt MQTT connection if WiFi is connected
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  Serial.print("Connecting to MQTT broker...");
  
  // Attempt to connect to MQTT broker with client ID
  if (mqttClient.connect(MQTT_CLIENT_ID)) {
    Serial.println(" ✓ Connected!");
    
    // Subscribe to command topic for remote control capability
    mqttClient.subscribe(MQTT_TOPIC_COMMAND);
    Serial.printf("  Subscribed to: %s\n", MQTT_TOPIC_COMMAND);
  } else {
    Serial.print(" ✗ Failed, rc=");
    Serial.println(mqttClient.state());  // Print error code for debugging
  }
}

// ========================================
// MQTT CALLBACK (Remote Commands)
// ========================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Convert payload bytes to String for easier processing
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];  // Append each character
  }
  
  Serial.printf("\n[MQTT] Command received: %s\n", message.c_str());

  // Process ARM command - enable security system remotely
  if (message == "ARM") {
    systemArmed = true;     // Enable security monitoring
    alarmActive = false;    // Clear any active alarms
    Serial.println("🔒 System ARMED remotely");
    blinkLED(3, 150);       // Visual confirmation
  } 
  // Process DISARM command - disable security system remotely
  else if (message == "DISARM") {
    systemArmed = false;           // Disable security monitoring
    alarmActive = false;           // Clear any active alarms
    digitalWrite(BUZZER_PIN, LOW); // Turn off buzzer immediately
    Serial.println("🔓 System DISARMED remotely");
    blinkLED(5, 100);              // Visual confirmation (different pattern)
  }
  // Process TEST_ALARM command - test alarm functionality
  else if (message == "TEST_ALARM") {
    soundAlarm(2);  // Sound alarm for 2 seconds
  }
  // Process STATUS command - force immediate status update
  else if (message == "STATUS") {
    publishStatus();  // Send current status immediately
  }
}

// ========================================
// READ ALL SENSORS
// ========================================
void readSensors() {
  // Read temperature and humidity from DHT22 sensor
  float h = dhtSensor.readHumidity();     // Get humidity reading
  float t = dhtSensor.readTemperature();  // Get temperature reading in Celsius

  // Check if readings are valid (not NaN - Not a Number)
  if (!isnan(h) && !isnan(t)) {
    currentTemp = t;       // Update temperature variable
    currentHumidity = h;   // Update humidity variable
  } else {
    Serial.println("⚠️ DHT sensor read error");  // Log error if reading failed
  }

  // Read digital sensors - PIR motion detector
  motionDetected = digitalRead(PIR_PIN) == HIGH;  // HIGH = motion detected
  
  // Read door sensor (reed switch with pull-up resistor)
  doorOpen = digitalRead(DOOR_PIN) == HIGH;  // Pull-up means HIGH = open, LOW = closed
  
  // Read arm/disarm button (with pull-up resistor)
  buttonPressed = digitalRead(ARM_BUTTON_PIN) == LOW;  // Pull-up means LOW = pressed
}

// ========================================
// ENVIRONMENTAL CONTROL (Hysteresis Logic)
// ========================================
void processEnvironmentalControl() {
  // Hysteresis cooling control prevents rapid on/off cycling
  // Turn fan ON if temp > 30°C, keep ON until temp < 28°C
  
  // Check if temperature exceeds high threshold AND fan is currently off
  if (currentTemp > TEMP_HIGH_THRESHOLD && !fanIsOn) {
    digitalWrite(RELAY_FAN_PIN, HIGH);  // Activate relay to turn fan ON
    fanIsOn = true;                     // Update fan state variable
    Serial.println("🌡️ HEAT WARNING! Fan STARTED");
    Serial.printf("   Temperature: %.1f°C (Threshold: %.1f°C)\n", 
                  currentTemp, TEMP_HIGH_THRESHOLD);
  } 
  // Check if temperature drops below low threshold AND fan is currently on
  else if (currentTemp < TEMP_LOW_THRESHOLD && fanIsOn) {
    digitalWrite(RELAY_FAN_PIN, LOW);  // Deactivate relay to turn fan OFF
    fanIsOn = false;                   // Update fan state variable
    Serial.println("❄️ Temperature normalized. Fan STOPPED");
    Serial.printf("   Temperature: %.1f°C (Threshold: %.1f°C)\n", 
                  currentTemp, TEMP_LOW_THRESHOLD);
  }
  // Note: If temp is between 28-30°C, fan state remains unchanged (hysteresis band)

  // Humidity warning (informational only, no action taken)
  static bool humidityWarningShown = false;  // Track if warning already displayed
  if (currentHumidity > HUMIDITY_WARNING && !humidityWarningShown) {
    Serial.printf("💧 High humidity detected: %.1f%%\n", currentHumidity);
    humidityWarningShown = true;  // Set flag to prevent repeated warnings
  } else if (currentHumidity <= HUMIDITY_WARNING) {
    humidityWarningShown = false;  // Reset flag when humidity normalizes
  }
}

// ========================================
// SECURITY SYSTEM (State Machine)
// ========================================
void processSecuritySystem() {
  systemStatus = "SECURE";  // Default status (no threats detected)
  
  // Only monitor for threats if system is armed
  if (systemArmed) {
    // Check for perimeter breach (door opened)
    if (doorOpen) {
      alarmActive = true;                     // Trigger alarm
      systemStatus = "🚨 BREACH: DOOR OPEN";  // Update status message
      
      if (!alarmActive) { // First detection (prevent log spam)
        Serial.println("\n!!! SECURITY BREACH DETECTED !!!");
        Serial.println("    Door sensor triggered");
      }
    }
    // Check for interior intrusion (motion detected) - only if door is closed
    else if (motionDetected) {
      alarmActive = true;                          // Trigger alarm
      systemStatus = "🚨 BREACH: INTERIOR MOTION"; // Update status message
      
      // Rate-limit logging to prevent serial monitor spam
      static unsigned long lastMotionLog = 0;
      if (millis() - lastMotionLog > 5000) {  // Log every 5 seconds max
        Serial.println("\n!!! SECURITY BREACH DETECTED !!!");
        Serial.println("    Motion sensor triggered");
        lastMotionLog = millis();  // Update timestamp
      }
    }
    else {
      // No threats detected - clear alarm if it was active
      if (alarmActive) {
        Serial.println("✓ Threat cleared");  // Log when threat is resolved
      }
      alarmActive = false;  // Deactivate alarm
    }
  } else {
    // System disarmed - ignore all sensors
    systemStatus = "DISARMED";  // Update status
    alarmActive = false;         // Ensure alarm is off
  }

  // Sound alarm if active and system is armed
  if (alarmActive && systemArmed) {
    static unsigned long lastBeep = 0;  // Track last beep time
    if (millis() - lastBeep > 1000) {   // Beep every second
      soundAlarm(1);                    // Sound alarm for 1 second
      lastBeep = millis();              // Update timestamp
    }
  } else {
    digitalWrite(BUZZER_PIN, LOW);  // Ensure buzzer is off when alarm not active
  }
}

// ========================================
// BUTTON HANDLER (Arm/Disarm Toggle)
// ========================================
void handleButtonPress() {
  // Check if button is pressed AND debounce time has passed (1 second)
  if (buttonPressed && (millis() - lastButtonPress > 1000)) {
    // Toggle armed state (true becomes false, false becomes true)
    systemArmed = !systemArmed;
    alarmActive = false;  // Reset alarm on state change
    lastButtonPress = millis();  // Update debounce timestamp

    // Log state change with visual separator
    Serial.println("\n========================================");
    Serial.printf("   SECURITY STATE CHANGED: %s\n", 
                  systemArmed ? "🔒 ARMED" : "🔓 DISARMED");
    Serial.println("========================================\n");

    // Visual feedback - blink LED
    blinkLED(3, 150);
    
    // Audible feedback - two short beeps
    for (int i = 0; i < 2; i++) {
      digitalWrite(BUZZER_PIN, HIGH);  // Turn buzzer on
      delay(100);                      // Keep on for 100ms
      digitalWrite(BUZZER_PIN, LOW);   // Turn buzzer off
      delay(100);                      // Keep off for 100ms
    }
  }
}

// ========================================
// ALARM SIREN
// ========================================
void soundAlarm(int duration) {
  // Create alarm pattern by rapidly toggling buzzer
  for (int i = 0; i < duration * 5; i++) {  // 5 cycles per second
    digitalWrite(BUZZER_PIN, HIGH);  // Turn buzzer on
    delay(100);                      // On for 100ms
    digitalWrite(BUZZER_PIN, LOW);   // Turn buzzer off
    delay(100);                      // Off for 100ms
  }
}

// ========================================
// BLINK LED
// ========================================
void blinkLED(int times, int delayMs) {
  bool originalState = digitalRead(STATUS_LED_PIN);  // Save current LED state
  
  // Blink specified number of times
  for (int i = 0; i < times; i++) {
    digitalWrite(STATUS_LED_PIN, HIGH);  // Turn LED on
    delay(delayMs);                      // Wait
    digitalWrite(STATUS_LED_PIN, LOW);   // Turn LED off
    delay(delayMs);                      // Wait
  }
  
  digitalWrite(STATUS_LED_PIN, originalState);  // Restore original LED state
}

// ========================================
// PUBLISH STATUS TO MQTT
// ========================================
void publishStatus() {
  // Don't attempt to publish if MQTT is disconnected
  if (!mqttClient.connected()) {
    return;
  }

  // Create JSON document with 512 bytes of memory
  StaticJsonDocument<512> doc;
  
  // Add environmental data to JSON
  doc["temperature"] = round(currentTemp * 10) / 10.0;      // Round to 1 decimal place
  doc["humidity"] = round(currentHumidity * 10) / 10.0;     // Round to 1 decimal place
  doc["fan_active"] = fanIsOn;                              // Fan state (boolean)
  
  // Add security data to JSON
  doc["armed"] = systemArmed;                // System armed state
  doc["alarm_active"] = alarmActive;         // Alarm triggered state
  doc["door_open"] = doorOpen;               // Door sensor state
  doc["motion_detected"] = motionDetected;   // Motion sensor state
  doc["status"] = systemStatus;              // Human-readable status message
  
  // Add system info to JSON
  doc["uptime"] = millis() / 1000;  // System uptime in seconds
  doc["rssi"] = WiFi.RSSI();        // WiFi signal strength (dBm)
  
  // Serialize JSON document to string
  String jsonString;
  serializeJson(doc, jsonString);

  // Publish to main status topic
  mqttClient.publish(MQTT_TOPIC_STATUS, jsonString.c_str());

  // Publish alarm state separately for easy monitoring/alerting
  if (alarmActive) {
    mqttClient.publish(MQTT_TOPIC_ALARM, "ACTIVE");
  }

  // Print formatted status to serial monitor for debugging
  Serial.println("\n[MQTT Published]");
  Serial.println(jsonString);
  Serial.println("─────────────────────────────────────────");
  Serial.printf("🌡️ Temp: %.1f°C | 💧 Humidity: %.1f%%\n", currentTemp, currentHumidity);
  Serial.printf("🌀 Fan: %s | 🔒 Armed: %s\n", 
                fanIsOn ? "ON " : "OFF", 
                systemArmed ? "YES" : "NO ");
  Serial.printf("🚪 Door: %s | 👁️ Motion: %s\n", 
                doorOpen ? "OPEN  " : "CLOSED", 
                motionDetected ? "YES" : "NO ");
  Serial.printf("📊 Status: %s\n", systemStatus.c_str());
  Serial.println("─────────────────────────────────────────\n");
}