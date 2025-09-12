# WRO Future Engineers 2025 – Complete Reference Vehicle

> *Comprehensive autonomous vehicle implementations for both Open Challenge and Obstacle Challenge rounds in WRO 2025 Future Engineers category, featuring single ESP32 and dual-controller (Raspberry Pi + ESP32) architectures.*

---

## Table of Contents

- [Overview](#overview)
- [Repository Structure](#repository-structure)
- [Team Members](#team-members)
- [Hardware](#hardware)
  - [Components](#components)
  - [Mobility System](#mobility-system)
  - [Power & Sensors](#power--sensors)
- [Software](#software)
  - [Setup & Initialization](#setup--initialization)
  - [Object Detection & Navigation](#object-detection--navigation)
  - [Driving Strategies & Challenge Logic](#driving-strategies--challenge-logic)
- [Assembly Instructions](#assembly-instructions)
- [Potential Improvements](#potential-improvements)
- [License / Acknowledgements](#license--acknowledgements)

---

## Overview

This project provides **comprehensive reference implementations** for both challenge rounds of the *WRO Future Engineers 2025* competition.  
It serves as a **complete learning platform** for participating teams and includes **multiple architectural approaches** for different competition strategies.  

## 🏆 Dual Challenge Support

### **Open Challenge Round**
- **Single ESP32 controller** for streamlined autonomous navigation
- Wall-following algorithms with ultrasonic sensors
- IMU-based orientation correction
- PlatformIO and Arduino IDE compatible implementations

### **Obstacle Challenge Round**  
- **Master-Slave architecture** (Raspberry Pi + ESP32)
- Advanced computer vision for obstacle detection
- Real-time image processing and decision making
- Region-based detection algorithms
- Serial communication between controllers

## ✨ Key Features  
- **Multiple Implementation Options:** Choose between single-controller (ESP32) or dual-controller (RPi+ESP32) setups
- **Complete Documentation:** Comprehensive wiring diagrams, assembly instructions, and code explanations
- **Modular Design:** Easily adaptable software for custom strategies and hardware configurations
- **Competition-Ready:** Full-mark scoring potential with proven navigation algorithms  
---

## Repository Structure

| Directory        | Description                                                     |
|------------------|-----------------------------------------------------------------|
| `src/`           | **Complete source code** for both Open and Obstacle rounds     |
| `schematics/`    | **Wiring diagrams** organized by competition round             |
| `models/`        | **3D CAD files** for vehicle parts, mounts, and brackets       |
| `vehicle_photos/`| **Multi-angle photos** of the completed vehicle build          |
| `video/`         | **Demo videos** and competition run recordings                  |
| `team_photo/`    | **Team member photos** and organization information             |
| `others/`        | **Additional resources** and documentation files               |

### 📁 Key Folder Details

#### **src/ - Source Code Repository**
- `ESP32_Vehicle/` - PlatformIO project for Open Round (single ESP32)
- `Open Round/` - Standalone code files for Open Challenge  
- `Obstacle Round/` - Master-Slave architecture (Raspberry Pi + ESP32)
- `main.py` - Reserved for future unified integration

#### **schematics/ - Wiring & Connection Diagrams** 
- `Open Round/` - ESP32-centric wiring diagrams
- `Obstacle Round/` - Dual-controller setup diagrams
- Complete pin assignments and component connections

#### **vehicle_photos/ - Build Reference Images**
- High-quality photos from 5 angles (front, back, left, right, top)
- Visual assembly and component placement reference

---

## Team Members
<p align="center">
  <img src="team_photo/Yolabs-logo.jpg" alt="Team Photo" width="500">
</p>

- **Siddharth Andhale** – Lead Design & Programming – [sidd1212004@gmail.com](mailto:sidd1212004@gmail.com)  
- **Team YoLabs** – WRO Future Engineers preparation and reference build development.  


---

## Hardware

### Architecture Options

The repository provides implementations for two distinct hardware architectures:

#### **🔧 Open Challenge Setup (Single Controller)**
| Component         | Description                             | Notes                                 |
|-------------------|-----------------------------------------|---------------------------------------|
| **ESP32 Dev Board** | Main and only controller              | Handles all processing and control   |
| **DC Motor**      | Rear-wheel drive with L298N driver     | PWM speed control via ESP32          |
| **Steering Servo** | SG90/MG996R front-wheel steering      | High-torque recommended               |
| **Ultrasonic Sensors** | Front & right distance measurement | HC-SR04 or equivalent               |
| **MPU6050 IMU**   | Orientation and drift correction        | I2C communication                     |
| **Power System**  | 2S/3S LiPo battery with BEC           | Regulated 5V for servo and sensors   |

#### **🔧 Obstacle Challenge Setup (Master-Slave)**
| Component         | Description                             | Notes                                 |
|-------------------|-----------------------------------------|---------------------------------------|
| **Raspberry Pi 4B** | Master controller (vision processing)| Main decision-making unit             |
| **ESP32 Dev Board** | Slave controller (motor/sensor control)| Real-time hardware interface         |
| **USB Camera**    | Computer vision input                   | Wide-angle lens, front-mounted       |
| **DC Motors**     | Enhanced drive system                   | Dual motors supported                 |
| **Steering Servo** | Precision front-wheel steering         | Servo feedback for accuracy          |
| **Multiple Sensors** | Ultrasonic + IMU sensor array        | Enhanced sensing capabilities         |
| **Enhanced Power** | Higher capacity battery system         | Supports dual controllers             |

### Components

#### **Mobility System**
- **Configuration:** Front-wheel steering with rear-wheel drive architecture
- **Turning Radius:** Optimized for WRO track specifications and tight corners  
- **Control Systems:** 
  - Open Round: Direct ESP32 PWM control
  - Obstacle Round: Raspberry Pi command → ESP32 execution
- **Design Rationale:** Realistic car-like dynamics ideal for competition simulation

#### **Power & Sensors**

##### **Open Round Power Management**
- **Motor Power:** 2S LiPo direct connection via L298N
- **Logic Power:** 5V BEC for ESP32 and sensors  
- **Servo Power:** Dedicated regulated supply to prevent brownouts

##### **Obstacle Round Power Management**  
- **Dual Controller Power:** Separate supplies for RPi (5V) and ESP32 (3.3V/5V)
- **Motor Power:** Enhanced current capacity for dual-motor configurations
- **Camera Power:** USB power delivery through Raspberry Pi

##### **Sensor Integration**
- **Ultrasonic Sensors:** Wall distance measurement and corner detection
- **IMU (MPU6050):** Real-time orientation correction and drift compensation  
- **Camera (Obstacle Round):** OpenCV-compatible USB camera for object detection  

---

## Software

### Architecture Overview

The repository provides complete software implementations for both competition rounds with different complexity levels:

#### **🔄 Open Challenge Round - Single Controller Architecture**
- **ESP32-Only Implementation:** Streamlined autonomous navigation
- **Direct Hardware Control:** No communication overhead
- **Multiple Development Options:** PlatformIO and Arduino IDE support
- **Optimized Performance:** Real-time sensor processing and motor control

#### **🔄 Obstacle Challenge Round - Master-Slave Architecture**  
- **Raspberry Pi (Master):** High-level vision processing and decision making
- **ESP32 (Slave):** Real-time motor control and sensor interface
- **Advanced Capabilities:** Computer vision, object detection, and complex navigation
- **Scalable Design:** Modular architecture for easy feature expansion

### Setup & Initialization

#### **Open Round Setup (ESP32 Only)**
1. **Hardware Preparation:**
   - Install PlatformIO or Arduino IDE
   - Connect ESP32 via USB
   
2. **Code Deployment Options:**
   - **PlatformIO Project:** Use `src/ESP32_Vehicle/` complete project
   - **Arduino IDE:** Copy code from `src/Open Round/esp32_code.cpp`
   
3. **Dependencies:** 
   ```cpp
   // Core libraries (usually pre-installed)
   #include <Wire.h>
   #include <Servo.h>
   // Additional: IMU library (MPU6050), Ultrasonic library
   ```

#### **Obstacle Round Setup (Master-Slave)**
1. **Raspberry Pi (Master) Setup:**
   ```bash
   # Install required Python packages
   pip install opencv-python numpy pyserial imutils
   
   # Clone and navigate to obstacle round code
   cd src/Obstacle\ Round/rpi_codes\ \(Master\)/
   ```

2. **ESP32 (Slave) Setup:**
   - Flash `src/Obstacle Round/esp32 code(Slave)/serial_receive_and_send_commands.cpp`
   - Configure serial communication pins
   - Establish UART connection with Raspberry Pi

3. **Integration:**
   - Serial communication at 115200 baud rate
   - Real-time command exchange between controllers
   - Coordinated sensor data sharing

### Object Detection & Navigation

#### **Open Round - Sensor-Based Navigation**
```cpp
// Core navigation logic
- Ultrasonic wall following with PD control
- IMU-based orientation correction  
- Corner detection and autonomous turning
- Lap completion tracking
```

#### **Obstacle Round - Vision-Based Navigation** 
```python
# Advanced computer vision pipeline
- OpenCV image processing (BGR → HSV conversion)
- Contour detection and object classification  
- Region-based obstacle analysis
- Dynamic path planning and avoidance
```

### Driving Strategies & Challenge Logic

#### **Open Round Strategies**
- **Wall Following:** PD-controlled steering using right ultrasonic sensor
- **Corner Detection:** Coordinated front ultrasonic + IMU-based turn initiation  
- **Speed Management:** Adaptive speed control based on proximity sensors
- **Lap Completion:** Distance tracking with IMU validation for accurate lap counting

#### **Obstacle Round Strategies**  
- **Master-Slave Coordination:** High-level decisions (RPi) → Low-level execution (ESP32)
- **Real-Time Processing:** Computer vision at 30+ FPS with optimized algorithms
- **Dynamic Obstacle Avoidance:** Region-based detection with predictive path planning
- **Recovery Protocols:** Multi-sensor fusion for error detection and correction

### Code Architecture

#### **File Organization**
```
src/
├── main.py                     # Future integration entry point
├── ESP32_Vehicle/              # PlatformIO Open Round project
│   ├── src/main.cpp           # Complete autonomous navigation
│   └── temp_files/            # Development variants
├── Open Round/                 # Standalone implementations  
│   ├── esp32_code.cpp         # ESP32 Arduino-compatible code
│   └── rpi_button_command.py  # Optional RPi interface
└── Obstacle Round/             # Master-Slave architecture
    ├── rpi_codes (Master)/    # Computer vision processing
    └── esp32 code(Slave)/     # Hardware control interface
```

---

## Assembly Instructions

### **🔧 General Assembly Process**

1. **Chassis Preparation:**  
   - Follow 3D models in `/models/` folder for custom parts
   - Mount motor and servo with proper alignment
   - Ensure adequate clearance for steering mechanism

2. **Controller Installation:**
   - **Open Round:** Single ESP32 mounting with vibration dampening
   - **Obstacle Round:** Dual mounting (RPi + ESP32) with proper ventilation
   - Secure all connections to prevent disconnection during operation

3. **Sensor Integration:**
   - Mount ultrasonic sensors at optimal angles (refer to `/vehicle_photos/`)
   - IMU placement: minimize vibration, maximize stability
   - **Obstacle Round:** Camera positioning for optimal field of view

4. **Power System Wiring:**
   - Follow wiring diagrams in `/schematics/` by competition round
   - Implement proper power distribution and regulation
   - Include emergency stop mechanisms

5. **Calibration & Testing:**
   - Servo angle calibration (typically 55°-132° range)
   - Sensor offset calibration in controlled environment  
   - Motor speed and direction validation
   - **Obstacle Round:** Camera calibration and focus adjustment

### **📐 Critical Assembly Notes**

- **GPIO Pin Verification:** Ensure code pin assignments match physical wiring
- **Power Requirements:** Verify voltage and current ratings for all components
- **Mechanical Constraints:** Reference `/vehicle_photos/` for proper component placement
- **Cable Management:** Route cables to avoid interference with moving parts
- **Testing Protocol:** Test each subsystem individually before full integration

## Quick Start Guide

### **🚀 Open Challenge Round**
1. Navigate to `/schematics/Open Round/` for wiring diagrams
2. Choose your development approach:
   - **PlatformIO:** Open `/src/ESP32_Vehicle/` project  
   - **Arduino IDE:** Use `/src/Open Round/esp32_code.cpp`
3. Flash ESP32 and calibrate sensors
4. Test wall-following and autonomous navigation

### **🚀 Obstacle Challenge Round**  
1. Review `/schematics/Obstacle Round/` for dual-controller wiring
2. Set up Raspberry Pi with camera and OpenCV dependencies
3. Flash ESP32 with slave controller code
4. Establish serial communication between controllers
5. Test computer vision and obstacle detection algorithms

## Performance & Competition Results

### **📺 Video Documentation**
- **Open Round Demo:** Available in `/video/` folder with YouTube link
- **Test Run Results:** Demonstrates successful autonomous navigation
- **Build Documentation:** Multi-angle photos in `/vehicle_photos/`

### **🏁 Competition Capabilities**
- **Open Round:** Proven wall-following and lap completion
- **Obstacle Round:** Advanced object detection and avoidance
- **Reliability:** Tested hardware configurations with backup strategies

---

## Potential Improvements & Future Development

### **🔮 Open Round Enhancements**
- **Enhanced Sensors:** Upgrade to ToF sensors for higher precision distance measurement
- **Improved Control:** Implement PID tuning interface for adaptive wall-following
- **Performance Optimization:** Code profiling and execution time optimization
- **Backup Navigation:** Redundant sensor systems for improved reliability

### **🔮 Obstacle Round Enhancements**  
- **Advanced Vision:** Integrate deep learning for more robust object classification
- **Sensor Fusion:** Combine LiDAR with camera for 3D obstacle mapping
- **Predictive Algorithms:** Machine learning for dynamic path optimization
- **Real-Time Performance:** GPU acceleration on Raspberry Pi for faster processing

### **🔮 System-Wide Improvements**
- **Unified Architecture:** Complete integration using `main.py` for seamless round switching
- **Auto-Calibration:** Self-calibrating sensor systems for easier deployment
- **Telemetry System:** Real-time monitoring and logging for performance analysis  
- **Modular Design:** Hot-swappable sensor modules for quick configuration changes
- **Competition Mode:** One-click setup for different track configurations

### **🔮 Hardware Upgrades**
- **Enhanced Processing:** Raspberry Pi 5 or dedicated AI accelerator integration
- **Precision Actuators:** Encoder feedback for motors and servo position verification  
- **Advanced Sensors:** Multi-beam LiDAR, stereo cameras, or radar integration
- **Power Management:** Intelligent battery management with charge monitoring

---

## License / Acknowledgements

This project is licensed under the [MIT License](LICENSE).  

Special thanks to:  
- [OpenCV](https://opencv.org/) – Computer vision processing.  
- [Raspberry Pi Foundation](https://www.raspberrypi.org/) – SBC platform.  
- WRO community and participating teams for continuous inspiration.

---

> For any questions or feedback, please feel free to contact us via email or open an issue on this repository.
