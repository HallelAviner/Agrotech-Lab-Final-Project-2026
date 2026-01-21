# Agrotech Lab Final Project 2026
A final project in the Agrotech Lab course at the Hebrew University. The goal of the project is to build a system for detecting leaks in an irrigation system.

# Irrigation system leakage sensor

## Introduction
In many cases, farmers are not in the field during irrigation. As a result, when a pipe becomes disconnected and the flow is too strong or when drippers are clogged they are not aware of the problem in real time. They only discover it later when they see the damage such as excessive water consumption or dry plants. To solve this we developed a system that prevents this lack of awareness by providing real time alerts as soon as a problem occurs.
This project involves the development of a smart irrigation monitor and control system based on the ESP32 microcontroller. The system is designed to detect real-time failures in the irrigation system using a flow sensor that measures the flow continuously and compares it to predefined normal values. The system operates on an anomaly detection logic based on a 20% deviation from the expected flow rate: in the event of high flow, indicating a leak or a burst pipe, the system sends an MQTT command to automatically close the main valve and issues a email notification via the Blynk app. In the case of low flow, indicating a clog, a 'weak flow' notification is sent to the user. To ensure energy efficiency, the controller enters Deep Sleep mode after ten minutes of inactivity and wakes up automatically when flow is detected. To overcome the logical challenge where the valve is closed and the controller is 'asleep' without flow to trigger it, a touch sensor was integrated. This allows the user to manually wake the controller and regain remote control of the valve through the app whenever necessary.

<img width="622" height="332" alt="image" src="https://github.com/user-attachments/assets/9b690fa3-4f3e-44c8-95c5-6ca7c034de10" />



## Hardware and assembly
### System Components Breakdown
| Component | Hardware Model | Key Function | Datasheet | image|
| :--- | :--- | :--- | :--- |:--- |
| **Microcontroller** | FireBeetle 2 ESP32-E | Acts as the central hub for data processing, WiFi communication (Blynk/MQTT), and power management. | [FireBeetle 2 ESP32-E](https://wiki.dfrobot.com/FireBeetle_Board_ESP32_E_SKU_DFR0654)| <img width="190" height="284" alt="image" src="https://github.com/user-attachments/assets/eba672f5-6f8a-47fb-9e12-5d251723da23" />|
| **Flow Sensor** | YF-S201 (Hall Effect) | Monitors water consumption in real-time by measuring flow velocity via pulse frequency. |[YF-S201 PDF](https://cdn-shop.adafruit.com/product-files/828/C898+datasheet.pdf)|<img width="209" height="203" alt="image" src="https://github.com/user-attachments/assets/23612cc4-39a7-41f5-90ab-b2ad317842e6" />|
| **Touch Sensor** | Exposed Capacitive Touch Wire | Leverages the ESP32 internal touch peripheral (GPIO 13) to trigger wake-up interrupts from Deep Sleep. ||<img width="313" height="138" alt="image" src="https://github.com/user-attachments/assets/36a17265-82f4-48db-8ba2-d621976317f9" />|
| **Power Source** |3.7V Li-ion Battery | Provides portable power with optimized longevity using hardware-level power-saving cycles. |[3.7V Li-ion Battery](https://www.google.com/search?q=https://www.lithium-polymer-battery.net/product/8000mah-3-7v-li-polymer-battery-lp806090/)|<img width="1292" height="1713" alt="image" src="https://github.com/user-attachments/assets/8aeeae2c-f436-479c-9580-311fac7df5d4" />|
| **Actuator** | Solenoid Valve | Electromechanical control of water flow, triggered remotely via MQTT protocols. ||<img width="240" height="323" alt="image" src="https://github.com/user-attachments/assets/b67d4fda-37ed-4462-a6a9-2388cbb369c1" />|
| **Resistor** | 4.7k Ω | Serves as a Pull-up resistor for the flow sensor signal line, preventing "floating" states and ensuring the ESP32 accurately counts every pulse from the YF-S201.| [Resistor 4.7kΩ Specs](https://www.seielect.com/catalog/sei-cf_cfm.pdf)| <img width="214" height="289" alt="image" src="https://github.com/user-attachments/assets/7ca8c334-7e04-4ee9-9de5-c3926f839063" />|




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
  leakAlertSent = false;
  lowAlertSent = false;
  lowFlowCounter = 0;
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
      
      float highThreshold = userExpectedFlow * 1.20;
      float lowThreshold = userExpectedFlow * 0.80;
  
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
## Instructions & Operation Guide
To get the project up and running, please follow the wiring diagram and configuration steps below:

### 1. Hardware Setup (Wiring)
Connect the components to the ESP32 board as shown in the diagram:

* **Water Flow Sensor:**
    * **Red Wire:** Connect to VCC (Power).
    * **Black Wire:** Connect to GND (Ground).
    * **Yellow Wire (Signal):** Connect to the designated GPIO pin on the controller.
    * **Important:** Add a **4.7kΩ** resistor between the signal line (yellow) and **3V3** (Pull-up).

* **Touch Sensor (Exposed Wire):** * Connect the green cable shown in the diagram to create a touch sensor.
    * **Keep its end exposed** to allow for touch-based wakeup.

* **Plumbing Integration:** * Connect the flow sensor to the pipe, ensuring it is in line with the pipe connected to the MQTT-controlled faucet.The system is designed to be installed alongside a standard irrigation controller of any type. It operates a dedicated valve, separate from the one managed by the irrigation controller. These two valves are connected in series, meaning that closing either valve will halt the water flow. While the main irrigation valve remains closed and only opens during scheduled cycles, our valve remains normally open, closing only when a leak is detected. Once the leak is repaired, our valve can be manually reopened via the Blynk app to resume the scheduled irrigation.
<img src="https://github.com/user-attachments/assets/078e5f23-f9db-4ea1-b5ed-ef31c5fb0efb" width="50%">

### 2. Software Configuration
Follow these steps to flash the firmware:

1.  **Install Libraries:** Ensure all required libraries (e.g., `PubSubClient` for MQTT, `Blynk`, and `WiFi`) are installed in your IDE.
2.  **Communication Details:** Open the code and fill in your WiFi credentials (`SSID`/`Password`) and MQTT Broker details in the relevant variables.
3.  **Calibration:** If necessary, adjust the `Calibration Factor` in the code to match your specific flow sensor model.
4.  **Upload Code:** Connect the ESP32 to your computer and upload the sketch.




## Experiment

### Methodology
When testing the system, we faced several constraints that prevented us from assembling and testing it in the same way it would work in an agricultural irrigation system. First, we did not have an irrigation controller, and we had only one valve that could be controlled electronically (via MQTT). Therefore, we were forced to connect our leak detection system to this single MQTT valve and use it not only for stopping the water when a leak is detected but also for starting and stopping the irrigation according to the schedule (which is generally the role of an irrigation controller). Consequently, unlike a system installed in series with a controller, the valve in our experiment remained closed between irrigation cycles.
Another constraint we took into account is that our sensor only measures flow accurately above 60 L/h. Since we lacked a drip system or sprinklers that meet this threshold, we used a partially open manual valve to simulate the flow demand of a full irrigation line. Additionally, we used other manual valves to simulate various operational scenarios and malfunctions to test the system's response.


### System Setup and Valve Configuration
To simulate real-world irrigation conditions and various malfunction scenarios, we established a baseline setup using four valves:

* **Valve 1 (Main Control):** Open when irrigation is active, Closed when the system is shut off.
* **Valve 2 (Flow Regulator):** Set to a **Partially Open** position to establish baseline flow.
* **Valve 3 (Clog Simulator):** Set to **Open**, used to simulate blockages.
* **Valve 4 (System Expansion Simulator):** Set to **Closed**, used to simulate the addition of an irrigation pipe.
<img src="https://github.com/user-attachments/assets/30b6b3ba-168d-4247-b611-0f55d43ae084" width="40%">


### User Interface
The system is monitored via the **Blynk IoT Platform**. 
* **Live Dashboard:** [Blynk.Cloud](https://blynk.cloud/) (Requires authenticated access).
* **Alert System:** Real-time push notifications and email alerts are managed via Blynk's event engine.
<img src="https://github.com/user-attachments/assets/0b4c914f-b78f-4324-8dec-6f5dd023e7b8" width="40%">



### Scenarios Tested

Five key scenarios tested to validate the system's logic and hardware integration:
Baseline Operation (Day 1): Establishing a normal flow rate and verifying real-time data logging to Blynk.
Manual Wake-up (Day 1+2): Using the ESP32 touch sensor to wake the controller from Deep Sleep to regain manual control.
System Expansion / Leak Detection (Day 2): Simulating a scenario where the irrigation network is expanded (increasing flow), but the farmer neglects to update the expected flow rate in the app. This tests the automated alert system and the valve shutdown logic, which triggers when the flow exceeds the expected rate by 20%. It also verifies the functionality of updating flow parameters via the app.
Clog Detection (Day 3): Simulating a blockage in the drippers to verify the "Weak Flow" notification logic when flow drops significantly below expected levels.
Automated Flow Wake-up (Day 3): Demonstrating the system's ability to "wake on flow" once irrigation is initiated by an external MQTT command.


## Detailed Experimental Logs

### Day 1: System Baseline and Manual Interaction
#### Scenarios Tested:
* **Normal System Operation**: A standard run to establish the "Expected Flow" baseline and verify that the ESP32 correctly logs data to Blynk.
* **Manual Wake-up via Touch Sensor**: Testing the integrated touch sensor's ability to wake the ESP32 from Deep Sleep.

> **Note:** While we used this to initiate irrigation during our tests, for a farmer, this feature serves to manually wake the system and regain remote control after an automated safety shutdown caused by a leak.

#### Timeline of Events:
The following table documents the specific actions and events recorded during the first day of testing to establish the system's baseline operation and power-saving features.

| Date | Time | Action / Event | Reason |
| :--- | :--- | :--- | :--- |
| 1/18/2026 | 13:15 |  Closed **Valve 1** via the **Blynk** app switch and set the expectedflow to 107 L/h | Setting the initial parameters |
| 1/18/2026 | 15:15 | Woke the **ESP32** via the touch sensor and opened **Valve 1** via **Blynk** | Starting the irrigation cycle |
| 1/18/2026 | 15:45 | Closed **Valve 1** via the **Blynk** app switch | Ending the irrigation cycle |
| 1/18/2026 | 15:55 | **ESP32** entered **Deep Sleep** mode and disconnected from **Blynk** | No water flow detected for 10 minutes |

#### Day 1 Visualizations:
<img src="https://github.com/user-attachments/assets/e12d35b1-84a5-47b2-8932-a51e8a71485c" width="50%">
<img src="https://github.com/user-attachments/assets/29b6bf15-9483-4bb2-af68-577d9a69c939" width="50%">




### Day 2: System Expansion and Anomaly Detection

#### Scenarios Tested:
* **System Expansion (Simulated Leak)**: A scenario where the farmer adds a new irrigation line but forgets to update the configuration in the app. The system detects the increased flow as an anomaly, triggers a "Leak detected" alert because of the high flow, and automatically closes the valve.
* **Updating the expected flow:** After the system expansion, the farmer changes the expected flow to match the actual flow. The 'leak detected' alert is no longer triggered.
#### Timeline of Events:
The following table documents the actions and events recorded during the second day, focusing on the system's ability to identify flow deviations and the manual recovery process.

| Date | Time | Action / Event | Reason |
| :--- | :--- | :--- | :--- |
| 1/19/2026 | 15:04 | Partially opened **Valve 3**. During plumbing adjustments, a 1 L/h flow was detected, waking the **ESP32**. | Simulating the addition of a new irrigation line without updating the "Expected Flow" in **Blynk**. |
| 1/19/2026 | 15:14 | **ESP32** entered **Deep Sleep** mode and disconnected from **Blynk**. | No water flow detected for 10 minutes. |
| 1/19/2026 | 15:15 | Woke the **ESP32** via the touch sensor and opened **Valve 1** via **Blynk**. | Starting the irrigation cycle. |
| 1/19/2026 | 15:16 | Flow reached 218 L/h; **Valve 1** was automatically closed via **MQTT**, and an email alert was sent to the farmer. | Flow rate exceeded the 20% deviation threshold from the "Expected Flow". |
| 1/19/2026 | 15:18 | Updated "Expected Flow" to 235 L/h and reopened **Valve 1** via the **Blynk** app. | Farmer received the alert, updated the threshold to match the new configuration, and resumed irrigation. |
| 1/19/2026 | 15:45 | Closed **Valve 1** via the **Blynk** app switch. | Ending the irrigation cycle. |
| 1/19/2026 | 15:56 | **ESP32** entered **Deep Sleep** mode and disconnected from **Blynk**. | No water flow detected for 10 minutes. |

#### Day 2 Visualizations:
<img src="https://github.com/user-attachments/assets/9765d4a3-4ddd-4663-90cd-53cb8dc91fc2" width="50%">
<img src="https://github.com/user-attachments/assets/47a82b93-cf37-4347-a074-6323f8deb3b0" width="90%">




### Day 3: Malfunctions and Automated Triggers

#### Scenarios Tested:
* **Clogged Pipeline Simulation**: Partially closing a secondary valve to simulate a blockage or clogged drippers. The system identifies the drop in flow rate and after 5 minutes issues a "Weak Flow" notification, without closing the valve.
* **Flow-Induced Wake-up**: Demonstrating the system's ability to wake up automatically from Deep Sleep the moment water flow is detected (e.g., when the main valve is opened via an external MQTT command).

#### Timeline of Events:
The following table documents the complete sequence of actions and events recorded during the final day of testing.

| Date | Time | Action / Event | Reason |
| :--- | :--- | :--- | :--- |
| 1/20/2026 | 15:10 | Closed **Valve 3** | Simulating a clog in one of the irrigation lines |
| 1/20/2026 | 15:15 | Opened **Valve 1** via **MQTT** while the **ESP32** was in Deep Sleep. The device immediately woke up from the flow and began measurements | Testing the "Wake on Flow" feature |
| 1/20/2026 | 15:21 | Flow rate of 129 L/h detected; a "Weak Flow" email alert was received | Low flow triggered by the simulated clog |
| 1/20/2026 | 15:25 | Opened **Valve 3** | Farmer received the alert and cleared the blockage |
| 1/20/2026 | 15:27 | Flow jumped to 321 L/h; a leak alert was received, and water flow was automatically stopped via **MQTT** | **Unexpected Malfunction**: Flow exceeded the expected baseline of ~218 L/h (see note below) |
| 1/20/2026 | 15:28 | Partially closed **Valve 3** and opened the valve via the **Blynk** app switch | Adjusting the manual valve to return flow to the "Expected Flow" range |
| 1/20/2026 | 15:29 | Flow stabilized at 214 L/h, matching the "Expected Flow" threshold | Irrigation cycle resumed normal operation |
| 1/20/2026 | 15:45 | Closed **Valve 1** via the **Blynk** app switch | Ending the irrigation cycle |
| 1/20/2026 | 15:56 | **ESP32** entered **Deep Sleep** mode and disconnected from **Blynk** | No water flow detected for 10 minutes |

> ***Note on the Unexpected Malfunction (15:27):** We hypothesize that the flow increase occurred because the positioning of Valve 2 or Valve 4 was inadvertently altered between test runs. As the system was installed in a public area, it is possible that the valves were accidentally disturbed, leading to a wider opening and higher flow rates. Additionally, fluctuations in water pressure—which were not monitored during this experiment—may have also contributed to the observed flow variations.*

#### Day 3 Visualizations:
<img src="https://github.com/user-attachments/assets/a7907692-1d6f-4e09-b5b5-44612524e6cf" width="90%">
<img src="https://github.com/user-attachments/assets/2cac51af-81c6-42b6-9d8c-485c60099bed" width="85%">




### Overall Data Analysis
<img src="https://github.com/user-attachments/assets/e2828d3b-f7a8-46ad-81aa-a1ce4ec69776" width="100%">


