# WRO Future Engineers 2025 – Reference Robot

> *Full-mark scoring autonomous vehicle design for the WRO 2025 Future Engineers category, using Raspberry Pi and ESP-based control platforms.*

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

This project is a **reference autonomous vehicle** designed to achieve full marks in the *WRO Future Engineers 2025* challenge.  
It serves as both a **learning platform** for participating teams and a **benchmark bot** for testing driving logic, wall-following algorithms, and challenge-specific maneuvers.  

Key features:  
- **Front-wheel steering** and **rear-wheel drive** design for realistic vehicle dynamics.  
- **Raspberry Pi 4B** for vision-based navigation and **ESP32** for direct motor/sensor control.  
- Dual ultrasonic sensors for accurate wall following.  
- IMU-based orientation stabilization.  
- Modular software for quick adaptation to custom strategies.  
---

## Repository Structure

| Directory        | Description                              |
|------------------|------------------------------------------|
| `models`         | 3D CAD files for mounts and brackets     |
| `others`         | Additional reference files or resources  |
| `schematics`     | Wiring diagrams for ESP32 & RPi setups   |
| `src`            | Source code (Python for RPi, C++/Arduino for ESP32) |
| `team_photo`     | Photos of the team members               |
| `vehicle_photos` | Images of the completed and in-progress robot builds |
| `video`          | Demo and competition run videos          |
| `README.md`      | Project documentation (this file)        |

---

## Team Members
<p align="center">
  <img src="team_photo/Yolabs-logo.jpg" alt="Team Photo" width="500">
</p>

- **Siddharth Andhale** – Lead Design & Programming – [sidd1212004@gmail.com](mailto:sidd1212004@gmail.com)  
- **Team Mentorship** – WRO Future Engineers preparation and reference build development.  


---

## Hardware

### Components

| Component         | Description                             | Notes                                 |
|-------------------|-----------------------------------------|---------------------------------------|
| Chassis           | Custom lightweight frame                | Optimized for stability & handling   |
| DC Motor + L298N  | Rear-wheel drive                        | Controlled via ESP32 PWM signals     |
| Steering Servo    | SG90/MG996R high-torque servo           | Mounted for front-wheel steering     |
| Raspberry Pi 4B   | Main processing unit                    | Runs vision & navigation algorithms  |
| USB Camera        | Vision input                            | Front-mounted, wide-angle lens       |
| ESP32 Dev Board   | Low-level control unit                  | Handles motor drivers & sensor input |
| MPU6050           | IMU for orientation feedback            | I2C connected to ESP32               |
| Ultrasonic Sensor (×2) | Distance measurement                | Mounted front & right for wall following |
| LiPo Battery      | 2S/3S LiPo                              | Powers motors & electronics (via BEC)|
| Misc.             | 3D-printed mounts, connectors, wiring  |                                        |


### Mobility System

- **Configuration:** Front-wheel steering with a single DC rear drive motor.  
- **Turning Radius:** Optimized for narrow WRO track corners.  
- **Control:** PWM-based speed control, servo-based steering.  
- **Build Choice Reasoning:** Offers realistic car-like dynamics, ideal for FE challenge simulation.  

**Possible enhancements:** Dual drive motors for more torque, lighter chassis for better acceleration.  

### Power & Sensors

- **Power:**  
  - 2S LiPo for motor power.  
  - 5V BEC for Raspberry Pi & sensors.  
  - Separate regulated supply for servo to avoid brownouts.  
- **Sensors:**  
  - **Ultrasonic (Front, Right):** Wall distance and corner detection.  
  - **MPU6050 IMU:** Orientation correction during navigation.  
  - **USB Camera:** Vision-based navigation and challenge detection.  

---

## Software

### Setup & Initialization

1. **Raspberry Pi:**
   - Flash Raspberry Pi OS (Lite or Full).
   - Enable SSH, I2C, and Camera support.
   - Install Python dependencies:  
     ```bash
     pip install opencv-python numpy imutils
     ```
   - Clone this repository:
     ```bash
     git clone https://github.com/yoprojects/WRO_Future_Engineers_ref.git
     ```
2. **ESP32:**
   - Open `/src/esp32/` in PlatformIO or Arduino IDE.
   - Flash the firmware to handle motor control & sensor readings.
   - Connect via UART/I2C to Raspberry Pi.

### Object Detection & Navigation

- **Vision Processing (OpenCV on RPi):**
  - Convert frames from BGR to HSV.
  - Apply masking & contour filtering to detect track lines or gates.
- **Sensor Integration:**
  - Ultrasonic sensors for precise wall-following.
  - IMU for orientation drift correction.

### Driving Strategies & Challenge Logic

- **Wall Following:** PD-controlled steering using right ultrasonic distance.
- **Corner Detection:** Front ultrasonic + camera-based detection to initiate turns.
- **Lap Completion:** IMU + distance tracking to ensure accurate lap counts.
- **Recovery Logic:** If bot drifts, slow down and re-center before resuming speed.

---

## Assembly Instructions

1. **Chassis Prep:**  
   - Mount motor and servo securely.  
2. **Steering Mechanism:**  
   - Calibrate servo angles (min = 55°, max = 132°).  
3. **Electronics Install:**  
   - Mount RPi & ESP32 on vibration-dampened standoffs.  
   - Connect motor driver, sensors, and camera.  
4. **Power Wiring:**  
   - Ensure regulated supply to Pi & servo.  
5. **Testing:**  
   - Test each component (motors, servo, sensors) individually.  
   - Run calibration scripts before full run.

---

## Potential Improvements

- Integrate LiDAR for higher-precision mapping.  
- Implement sensor fusion for smoother path control.  
- Improve computer vision FPS via GPU acceleration on RPi.  
- Add auto-calibration for ultrasonic sensors.  
- Experiment with reinforcement learning for adaptive driving.

---

## License / Acknowledgements

This project is licensed under the [MIT License](LICENSE).  

Special thanks to:  
- [OpenCV](https://opencv.org/) – Computer vision processing.  
- [Raspberry Pi Foundation](https://www.raspberrypi.org/) – SBC platform.  
- WRO community and participating teams for continuous inspiration.

---

> For any questions or feedback, please feel free to contact us via email or open an issue on this repository.
