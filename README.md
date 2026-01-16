# Work in Progress - Agrotech-Lab-Final-Project-2026
A final project in the Agrotech Lab course at the Hebrew University. The goal of the project is to build a system for detecting leaks in an irrigation system.

# שם הפרויקט שלכם (למשל: מערכת השקיה חכמה מבוססת לחות קרקע)

## Introduction
In many cases, farmers are not in the field during irrigation. As a result, when a pipe becomes disconnected and the flow is too strong or when drippers are clogged they are not aware of the problem in real time. They only discover it later when they see the damage such as excessive water consumption or dry plants. To solve this we developed a system that prevents this lack of awareness by providing real time alerts as soon as a problem occurs.
This project involves the development of a smart irrigation monitor and control system based on the ESP32 microcontroller. The system is designed to detect real-time failures in the irrigation system using a flow sensor that samples the flow at set intervals and compares it to predefined normal values. The system operates on an anomaly detection logic based on a 15% deviation from the expected flow rate: in the event of high flow, indicating a leak or a burst pipe, the system sends an MQTT command to automatically close the main valve and issues a email notification via the Blynk app. In the case of low flow, indicating a clog, a 'weak flow' notification is sent to the user. To ensure energy efficiency, the controller enters Deep Sleep mode after ten minutes of inactivity and wakes up automatically when flow is detected. To overcome the logical challenge where the valve is closed and the controller is 'asleep' without flow to trigger it, a touch sensor was integrated. This allows the user to manually wake the controller and regain remote control of the valve through the app whenever necessary.


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
כאן נשים את הקוד. בגיטהאב אפשר פשוט להעלות את הקבצים, אבל ב-README כדאי להסביר עליו בקצרה.

## Instructions
איך מרכיבים ואיך משתמשים? (צעד אחר צעד).

## Experiment
כאן המקום להראות את הפרויקט "בפעולה".
* צילומי מסך מה-ThingSpeak שלכם.
* גרפים של הנתונים שנאספו (טמפרטורה, לחות וכו').
