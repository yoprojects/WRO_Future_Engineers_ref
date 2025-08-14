# WRO Future Engineer Vehicle – Open Round Code (v01)

This folder contains the **prototype reference code** for the WRO Future Engineers 2025 **Open Challenge Round** using the ESP32 microcontroller.  
The code implements autonomous navigation logic for the **reference vehicle**, including steering, motor control, and sensor integration.

---

## 📂 File Overview

| File | Description |
|------|-------------|
| **main.cpp** | Main executable code for the Open Challenge Round (PlatformIO format) |
| **temp_files/** | Backup versions and experimental variations of main.cpp for testing specific scenarios (e.g., right turns with/without stop, left+right logic) |

---

## ⚙️ Hardware Requirements
- **ESP32 Dev Board** (ESP-WROOM-32)
- DC motor with **L298N** motor driver (rear wheel drive)
- Servo motor for steering
- IMU (e.g., MPU6050)
- Ultrasonic sensors (front & right)
- Power supply (Li-ion battery or equivalent)

---

## 🚀 How to Use

### **Option 1 – Using PlatformIO (Recommended)**
1. Install [PlatformIO IDE](https://platformio.org/) (VS Code extension or standalone).
2. Open the project folder `ESP32_Vehicle` in PlatformIO.
3. Connect your ESP32 board via USB.
4. In `platformio.ini`, confirm your board setting (e.g., `board = esp32dev`).
5. Build & Upload:
   ```bash
   pio run --target upload
6. Monitor Serial Output:
   ```bash
   pio device monitor

### **Option 2 – Using Arduino IDE**

If you prefer not to use PlatformIO, you can still use the same logic from `main.cpp` in Arduino IDE:

1. **Install Arduino IDE** (latest version).

2. **Install ESP32 Board Package**:  
   - Go to **File → Preferences → Additional Board Manager URLs**  
   - Add:  
     ```
     https://dl.espressif.com/dl/package_esp32_index.json
     ```
   - Open **Tools → Board Manager** and install **ESP32 by Espressif Systems**.

3. **Install required libraries**:
   - `Wire` (usually pre-installed)
   - Servo control library (`ESP32Servo.h` or similar)
   - IMU library (e.g., `Adafruit MPU6050` or equivalent)
   - Ultrasonic sensor library (e.g., `NewPing` or similar)

4. **Copy the code**:  
   Copy the contents of `main.cpp` into a new Arduino `.ino` file.

5. **Select your board**:  
   - **Tools → Board → ESP32 Arduino → ESP32 Dev Module**

6. **Upload the code** using the Upload button.

7. **Open Serial Monitor** to view debug output.

---
## 📊 Control Flow – main.cpp

The following diagram illustrates the decision-making process used by the vehicle during autonomous operation.

```mermaid
flowchart LR

A[Start] --> B[Wait for Button Press]
B --> C[Setup Hardware & Sensors]
C --> D[Calibrate IMU & Record Start Pos]
D --> E[Main Loop]
E --> F[Update Sensors]
F --> G[Log Data]
G --> H{Returning to Start?}
H -- Yes --> I{At Start Pos?}
I -- Yes --> J[Stop & End]
I -- No --> K[Navigate]
H -- No --> K
K --> L[Check Lap Completion]
L --> M{Front < 25cm?}
M -- Yes --> N[Move Backward]
M -- No --> O{Front < 75 & Right > 90?}
O -- Yes --> P[Right Turn 90°]
P --> Q[Forward 300ms]
Q --> F
O -- No --> R{Front < 70 & Right < 75?}
R -- Yes --> S[Left Turn 133°]
S --> Q
R -- No --> T{Right > 90?}
T -- Yes --> U[Straight Forward]
T -- No --> V[Default Forward]
U --> F
V --> F
```
---

📌 **Notes**
- `temp_files/` contains alternative navigation logic for specific testing scenarios.
- Ensure that GPIO pins in the code match your actual wiring.
- Always calibrate the servo angles and motor speed before running the full challenge track.

---

🏷 **Version**  
v01 – Initial prototype code for Open Round (reference vehicle).

📄 For full wiring diagrams, see the `/schematics` folder in the repository.


---
