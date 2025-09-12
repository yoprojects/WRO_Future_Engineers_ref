# WRO Future Engineers - Wiring Schematics & Diagrams

This folder contains comprehensive wiring and connection diagrams for the **WRO Future Engineers Reference Build**, organized by competition round and development phase.

## 📂 Folder Structure

### **Open Round/**
- **WRO_FE_Scem.pdf** – Complete schematic for Open Challenge Round ESP32-based autonomous vehicle
- **README.md** – Specific documentation for Open Round wiring setup

### **Obstacle Round/**
- **WRO_FE_Scem_Obstacle_Round.pdf** – Advanced schematic for Obstacle Challenge Round with Master-Slave architecture (Raspberry Pi + ESP32)
- **README.md** – Specific documentation for Obstacle Round wiring setup

### **temp/**
- Development and experimental files (archived)

## 🚀 Quick Start Guide

### **For Open Round (Single ESP32 Setup)**
1. Navigate to `Open Round/` folder
2. Download and open `WRO_FE_Scem.pdf`
3. Follow the ESP32-centric wiring diagram
4. Refer to `/src/ESP32_Vehicle/` or `/src/Open Round/` for corresponding code

### **For Obstacle Round (Master-Slave Setup)**
1. Navigate to `Obstacle Round/` folder  
2. Download and open `WRO_FE_Scem_Obstacle_Round.pdf`
3. Follow the dual-controller wiring (Raspberry Pi + ESP32)
4. Refer to `/src/Obstacle Round/` for corresponding code

### **For Development Reference**
- Check `temp/` folder for archived development files

## 📋 Hardware Components by Round

### **Open Round Requirements**
- ESP32 Development Board
- DC Motor + L298N Driver
- Servo Motor (steering)
- Ultrasonic Sensors (2x)
- IMU Sensor (MPU6050)
- Power Supply System

### **Obstacle Round Requirements**  
- Raspberry Pi 4 (Master)
- ESP32 Development Board (Slave)
- Camera Module (Pi Camera or USB)
- DC Motors + Motor Drivers
- Servo Motor (steering)
- Multiple Ultrasonic Sensors
- IMU Sensor
- Serial Communication Interface
- Enhanced Power Supply

## 🔗 Related Documentation

- **Source Code**: `/src/` - Complete code implementations for both rounds
- **Vehicle Photos**: `/vehicle_photos/` - Physical build reference images  
- **Models**: `/models/` - 3D models and mechanical designs
- **Competition Videos**: `/video/` - Performance demonstrations

## 📄 File Access

### **Competition-Ready Schematics**
- [📄 Open Round Schematic](Open%20Round/WRO_FE_Scem.pdf) 
- [📄 Obstacle Round Schematic](Obstacle%20Round/WRO_FE_Scem_Obstacle_Round.pdf)

### **Development Reference**
- `temp/` folder contains archived development files

---

### 📌 Important Notes
- **Pin Assignments**: Ensure GPIO pins in schematics match your code implementation
- **Power Requirements**: Check voltage and current ratings for all components
- **Sensor Placement**: Follow mechanical constraints shown in vehicle photos
- **Cable Management**: Plan wire routing to avoid interference with moving parts

### 🏷️ Version History
- **v02** – Competition-ready schematics separated by challenge round
