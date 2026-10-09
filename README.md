# BLE Fall Detection System

A real-time fall detection and monitoring system built with the **Arduino Nano 33 BLE Rev2** and a **Web Bluetooth API** dashboard. The system continuously monitors motion data using on-board IMU sensors, detects free-fall events using raw acceleration thresholds, triggers local visual and audible alarms, and transmits real-time telemetry and alerts over Bluetooth Low Energy (BLE) to a browser interface.

---

## Features

* **Real-time Free-Fall Detection**: Uses raw acceleration magnitude to identify free-fall conditions without filter lag.


* **Local Visual & Audible Alarms**: Activates a buzzer, LED, and vibration motor upon confirmed fall detection.


* **Physical & Remote Alarm Reset**: Alarms can be cancelled via a hardware push button or remotely from the web dashboard over BLE.


* **BLE Data Streaming**: Transmits 6-axis IMU data ($\text{m/s}^2$ for acceleration, $^\circ/\text{s}$ for gyroscope) at approximately 50 Hz.


* **Web Bluetooth Dashboard**: A browser-based interface requiring no additional client installation.


* **3D Orientation Display**: Real-time canvas projection of device Pitch and Yaw.


* **6-Axis Telemetry Graphs**: Live canvas plots for $A_x, A_y, A_z$ and $G_x, G_y, G_z$ with configurable sample windows.


* **Browser & Haptic Alerts**: Push notifications and device vibration triggered on the client device during fall events.



---

## Hardware Requirements & Pinout

### Hardware Components

* **Microcontroller**: Arduino Nano 33 BLE Rev2 (with integrated BMI270 & BMM150 IMU)


* **Output Actuators**: Active Buzzer, LED indicator, Vibration Motor


* **Passive Components**: 220 Ohm resistor for LED


* **Input**: Push button



### Pin Configuration

| Pin | Component | Configuration / Notes |
| --- | --- | --- |
| **D5** | Vibration Module | Digital Output

 |
| **D6** | Active Buzzer | Digital Output

 |
| **D7** | LED | Digital Output via 220 Ohm resistor

 |
| **D8** | Push Button | Input with Internal Pull-up (connected to GND)

 |

---

## BLE Specifications

* **Device Name**: `FallDetector`

* **Service UUID**: `19b10000-e8f2-537e-4f6c-d104768a1214`


### Characteristics

| Characteristic | UUID | Properties | Format / Payload |
| --- | --- | --- | --- |
| **Alert** | `19b10001-e8f2-537e-4f6c-d104768a1214` | Read, Notify | String up to 30 bytes (e.g., `"SYSTEM READY"`, `"FALL DETECTED"`, `"FALL CANCELLED"`)

 |
| **Data** | `19b10002-e8f2-537e-4f6c-d104768a1214` | Read, Notify | String up to 80 bytes (`"DATA,ax,ay,az,gx,gy,gz"`)

 |
| **Command** | `19b10003-e8f2-537e-4f6c-d104768a1214` | Write | String up to 20 bytes (e.g., `"CANCEL"`)

 |

---

## Fall Detection & Signal Processing Logic

1. **Free-Fall Condition**: Free-fall is detected when total acceleration magnitude $\sqrt{a_x^2 + a_y^2 + a_z^2}$ falls below **$3.5\,\text{m/s}^2$** (~$0.255\,g$).


2. **Time Threshold**: The sub-threshold acceleration must persist continuously for at least **70 ms** (~3 samples at 50 Hz) to confirm a fall event.


3. **Dual Processing Pipeline**:
* **Raw Data**: Processed directly for fall detection logic to prevent delay or dampening from filtering.


* **Alpha Filtered Data ($\alpha = 0.05$)**: Used for BLE streaming to produce smooth visual graph lines without impacting detection reactivity.




4. **Alarm Trigger**: Upon fall confirmation, pins D5, D6, and D7 are set `HIGH` and an alert packet `"FALL DETECTED"` is broadcasted over BLE.


5. **Alarm Cancellation**:
* **Hardware**: Pressing the push button on D8 grounds the pin and resets state.


* **Software**: Sending `"CANCEL"` command via BLE resets alarms and broadcasts `"FALL CANCELLED"`.





---

## Web Monitor Dashboard

The web interface is written in HTML/CSS/JS and connects directly to the device using the **Web Bluetooth API**.

### Dashboard Capabilities

* **Connection Management**: Buttons to scan, pair, connect, disconnect, or pause data streaming.


* **3D Orientation Canvas**: Renders device orientation based on accelerometer Pitch and integrated gyroscope Yaw (Roll is intentionally ignored).


* **Telemetry Visualizer**: Displays 6 live graphs ($A_x, A_y, A_z, G_x, G_y, G_z$) with selectable window lengths (100, 250, 500, or 1000 samples).


* **Emergency Alert Bar**: Changes dynamically to a high-contrast red flashing banner during confirmed falls and provides a one-click remote alarm cancel button.


* **Client Notifications**: Triggers native browser push notifications and device vibration patterns (`navigator.vibrate`) upon detecting a fall alert.



---

## Setup & Usage Instructions

### 1. Firmware Installation

1. Install **Arduino IDE** (or VS Code with Arduino extension).
2. Install the required libraries via the Arduino Library Manager:
* `Arduino_BMI270_BMM150`

* `ArduinoBLE`



3. Select board **Arduino Nano 33 BLE**.


4. Compile and upload `fall_detector.ino` to the board.



### 2. Web Monitor Setup

1. Open `index.html` in a Web Bluetooth supported web browser (e.g., Google Chrome, Microsoft Edge).


2. Click **Connect BLE** and select **FallDetector** from the Bluetooth pairing prompt.


3. View real-time telemetry graphs, 3D orientation, and fall status alerts.
