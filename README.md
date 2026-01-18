# Agrotech Lab Final Project 2026
A final project in the Agrotech Lab course at the Hebrew University. The goal of the project is to build a system for detecting leaks in an irrigation system.

# Irrigation system leakage sensor

## Introduction
In many cases, farmers are not in the field during irrigation. As a result, when a pipe becomes disconnected and the flow is too strong or when drippers are clogged they are not aware of the problem in real time. They only discover it later when they see the damage such as excessive water consumption or dry plants. To solve this we developed a system that prevents this lack of awareness by providing real time alerts as soon as a problem occurs.
This project involves the development of a smart irrigation monitor and control system based on the ESP32 microcontroller. The system is designed to detect real-time failures in the irrigation system using a flow sensor that samples the flow at set intervals and compares it to predefined normal values. The system operates on an anomaly detection logic based on a 20% deviation from the expected flow rate: in the event of high flow, indicating a leak or a burst pipe, the system sends an MQTT command to automatically close the main valve and issues a email notification via the Blynk app. In the case of low flow, indicating a clog, a 'weak flow' notification is sent to the user. To ensure energy efficiency, the controller enters Deep Sleep mode after ten minutes of inactivity and wakes up automatically when flow is detected. To overcome the logical challenge where the valve is closed and the controller is 'asleep' without flow to trigger it, a touch sensor was integrated. This allows the user to manually wake the controller and regain remote control of the valve through the app whenever necessary.


## Hardware and assembly
### System Components Breakdown
| Component | Hardware Model | Key Function | Datasheet | 
| :--- | :--- | :--- | :--- |
| **Microcontroller** | FireBeetle 2 ESP32-E | Acts as the central hub for data processing, WiFi communication (Blynk/MQTT), and power management. | [FireBeetle 2 ESP32-E](https://wiki.dfrobot.com/FireBeetle_Board_ESP32_E_SKU_DFR0654)| 
| **Flow Sensor** | YF-S201 (Hall Effect) | Monitors water consumption in real-time by measuring flow velocity via pulse frequency. |[YF-S201 PDF](https://cdn-shop.adafruit.com/product-files/828/C898+datasheet.pdf)| 
| **Manual Trigger** | Exposed Capacitive Touch Wire | Leverages the ESP32 internal touch peripheral (GPIO 13) to trigger wake-up interrupts from Deep Sleep. || 
| **Power Source** | 3.7V Li-ion Battery | Provides portable power with optimized longevity using hardware-level power-saving cycles. || 
| **Actuator** | Solenoid Valve | Electromechanical control of water flow, triggered remotely via MQTT protocols. || 



## Code
```cpp
/*
 * Irrigation Leakage Detection System
 * This project uses an ESP32 to monitor water flow via a pulse-based sensor.
 * It integrates with Blynk for monitoring and uses MQTT to control greenhouse valves.
 * Features: Deep Sleep for power saving, Touch Wakeup, and Emergency Shutdown.
 */

#define BLYNK_PRINT Serial 
#define BLYNK_TEMPLATE_ID "TMPL6o8xDKv3p"
#define BLYNK_TEMPLATE_NAME "Leakage sensor"
#define BLYNK_AUTH_TOKEN "cb5feGxO9_Q_tZmrjvtiY9xRtV-ijtab"

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <PubSubClient.h> 

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// --- Network Credentials ---
char ssid[] = ""; // Your WiFi network name (SSID)
char pass[] = ""; // Your WiFi network password

// --- MQTT Configuration ---
const char* mqtt_server = "";          // IP address or hostname of the MQTT broker
const int mqtt_port = 1883;            // Port number for MQTT communication (default is 1883)
const char* mqtt_user = "";            // Username for MQTT broker authentication
const char* mqtt_password = "";        // Password for MQTT broker authentication
const char* mqtt_topic_solenoid = ""; // MQTT topic address used to control solenoid

// --- State Variables ---
bool lastIrrigationSwitch = false; 
bool currentIrrigationSwitch = false;

// --- Hardware and Flow Calibration ---
const int flowSensorPin = 25; // Connected to D2 on FireBeetle 2
volatile unsigned long pulseCount = 0;
unsigned long lastMeasureTime = 0;
unsigned long lastActivityDetected = 0;
bool zeroSent = false; // Tracks if a 0 flow value was sent after flow stopped

// --- RTC Variables (Preserved during Deep Sleep) ---
RTC_DATA_ATTR bool leakAlertSent = false;
RTC_DATA_ATTR bool lowAlertSent = false;
RTC_DATA_ATTR int lowFlowCounter = 0;
RTC_DATA_ATTR int userExpectedFlow = 0; 

// --- Timing and Sensitivity Settings ---
float calibrationFactor = 838.0; 
float measureInterval = 60000; // Frequency of reporting to Blynk (ms)
float sleepTimeout = 600000;   // Inactivity period before entering Deep Sleep (ms)

// --- Touch Sensor Configuration ---
const int touchSensorPin = T4; // GPIO 13 (D7)
int touchSensorThreshold = 500;


void IRAM_ATTR pulseCounter() {
  pulseCount++;
}

// Sync "Expected Flow" (L/h) from Blynk
BLYNK_WRITE(V1) {
  userExpectedFlow = param.asInt(); 
  Serial.print("New Regular Flow Setting: ");
  Serial.print(userExpectedFlow);
  Serial.println(" L/h");
}

// Control the Solenoid Switch from Blynk
BLYNK_WRITE(V2) {
  currentIrrigationSwitch = param.asInt();
  Serial.print("Blynk Switch (V2) updated to: ");
  Serial.println(currentIrrigationSwitch);
}


void enterDeepSleep() {
  // Allow time for final communications to finish
  delay(measureInterval / 10);
  Serial.println("Preparing for Deep Sleep...");
  
  // Clear persistent flags and disconnect
  leakAlertSent = false;
  lowAlertSent = false;
  lowFlowCounter = 0;
  
  Blynk.virtualWrite(V0, 0); // Force graph to 0 before sleeping
  Blynk.run();
  delay(100); 
  Blynk.disconnect();

  // 1. Setup Touch Wakeup
  touchSleepWakeUpEnable(touchSensorPin, touchSensorThreshold);
  esp_sleep_enable_touchpad_wakeup(); 

  // 2. Setup Flow Sensor Wakeup (EXT1)
  uint64_t pin_mask = 1ULL << flowSensorPin; 
  int currentState = digitalRead(flowSensorPin);
  
  // Logic: wake up when the current state changes
  if (currentState == LOW) {
    esp_sleep_enable_ext1_wakeup(pin_mask, ESP_EXT1_WAKEUP_ANY_HIGH);
  } else {
    esp_sleep_enable_ext1_wakeup(pin_mask, ESP_EXT1_WAKEUP_ALL_LOW);
  }

  Serial.println("Wakeup sources configured. Entering Deep Sleep.");
  Serial.flush(); 
  esp_deep_sleep_start();
}


void reconnectMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32_Leakage_Project";
    
    if (mqttClient.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
      Serial.println("Connected to Greenhouse MQTT");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" - retrying in 5 seconds...");
      delay(5000);
    }
  }
}


void setup() {
  Serial.begin(115200);
  pinMode(flowSensorPin, INPUT_PULLUP);
  
  Serial.println("Starting Blynk...");
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  
  // Synchronize settings from the server upon wake up
  Blynk.syncVirtual(V1);  
  Blynk.syncVirtual(V2);  

  mqttClient.setServer(mqtt_server, mqtt_port);
  
  // Warm up the touch sensor and attach flow interrupt
  touchRead(touchSensorPin);
  attachInterrupt(digitalPinToInterrupt(flowSensorPin), pulseCounter, CHANGE);
  
  lastMeasureTime = millis();
  lastActivityDetected = millis();
}


void loop() {
  Blynk.run();
  
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  mqttClient.loop();

  // Process manual irrigation switch changes
  if (currentIrrigationSwitch != lastIrrigationSwitch) {
    if (currentIrrigationSwitch == 1) {
      mqttClient.publish(mqtt_topic_solenoid, "1");
      Serial.println("MQTT Command: Open Solenoid 6");
    } else {
      mqttClient.publish(mqtt_topic_solenoid, "0");
      Serial.println("MQTT Command: Close Solenoid 6");
    }
    lastIrrigationSwitch = currentIrrigationSwitch;
    lastActivityDetected = millis(); 
  }

  // Periodic flow measurement and alert logic
  if (millis() - lastMeasureTime >= measureInterval) {
    unsigned long currentPulses = pulseCount;
    pulseCount = 0;
    
    // Calculate Flow Rate in Liters per Hour (L/h)
    float rawFlowRate = (currentPulses / calibrationFactor) * 3600000.0 / measureInterval;
    int flowRatePerHour = round(rawFlowRate); 

    if (currentPulses > 0) {
      Serial.print("Flow Rate: "); Serial.print(flowRatePerHour); Serial.println(" L/h");
      Blynk.virtualWrite(V0, flowRatePerHour);

      lastActivityDetected = millis();
      zeroSent = false;
      
      float highThreshold = userExpectedFlow * 1.15;
      float lowThreshold = userExpectedFlow * 0.85;
  
      // --- Alert Logic 1: High Flow (Leak) ---
      if (flowRatePerHour > highThreshold && !leakAlertSent) {
        float percentExceeded = ((float)(flowRatePerHour - userExpectedFlow) / userExpectedFlow) * 100.0;
        String alertMsg = String(flowRatePerHour) + " L/h (" + String(percentExceeded, 1) + "% high)";
        Blynk.logEvent("leak_detected", alertMsg);
        leakAlertSent = true; 
        
        mqttClient.publish(mqtt_topic_solenoid, "0"); // Emergency cutoff
        Serial.println("ALARM: High Flow detected! Solenoid 6 closed.");
      }
      else if (flowRatePerHour <= userExpectedFlow) {
        leakAlertSent = false; 
      }

      // --- Alert Logic 2: Low Flow (Blockage) ---
      if (flowRatePerHour < lowThreshold && !lowAlertSent) {
        if (lowFlowCounter < 5) {
          lowFlowCounter++;
        } else {
          float percentLower = ((float)(userExpectedFlow - flowRatePerHour) / userExpectedFlow) * 100.0;
          String alertMsg = String(flowRatePerHour) + " L/h (" + String(percentLower, 1) + "% low)";
          Blynk.logEvent("low_flow_alert", alertMsg);
          lowAlertSent = true;
          Serial.println("ALARM: Low Flow alert triggered.");
          lowFlowCounter = 0;
        }
      } else {
        lowFlowCounter = 0;
      }

      if (userExpectedFlow <= flowRatePerHour) {
        lowAlertSent = false;
      }
    }
    else {
      // No water movement detected
      Serial.println("Monitoring: No flow detected...");
      leakAlertSent = false;
      lowAlertSent = false;
      
      if (!zeroSent) {
        Blynk.virtualWrite(V0, 0);
        zeroSent = true;
      }
    }
    lastMeasureTime = millis();
  }

  // Manage Deep Sleep entry
  if (millis() - lastActivityDetected > sleepTimeout) {
    enterDeepSleep();
  }
}
```
## Instructions
איך מרכיבים ואיך משתמשים? (צעד אחר צעד).

## Experiment
כאן המקום להראות את הפרויקט "בפעולה".
* צילומי מסך מה-ThingSpeak שלכם.
* גרפים של הנתונים שנאספו (טמפרטורה, לחות וכו').
