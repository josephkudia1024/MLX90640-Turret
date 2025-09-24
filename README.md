# NERF Auto-Tracking Turret

An autonomous, vision-guided NERF blaster turret built from the ground up — featuring custom CAD design, thermal vision target detection, dual-core real-time control, and precision stepper-driven motion.  

This repository contains the **firmware** powering the system, designed for portability across different turret builds and hardware setups.

---

## 📂 Repository Contents

- `src/` – Source code for ESP32 firmware  
- `docs/` – Parts list, assembly/wiring diagrams, schematics, and design notes  
- `hardware/` – CAD models and reference design files

---

## 🔧 Hardware Architecture

- **Custom Structural Design**  
  - All structural components modeled in *AutoCAD Fusion*.  
  - Multiple design iterations tested for durability, stability, and cable routing.  
  - Entirely original **mechanical** and **software** integration.  

- **Power & Hardware**  
  - Central **24 V power supply** with **buck converters** regulating voltage for:  
    - Stepper motors  
    - ESP32 microcontroller  
    - Modified NERF blaster  
  - **Slip-ring integration** allows for full **360° continuous yaw rotation** while maintaining a hardwired wall connection (no reliance on onboard batteries).  

- **Control System**  
  - Dual **NEMA17 stepper motors** (pan/tilt) driven by **TB6600 drivers** for precise pitch/yaw articulation.  
  - Firing mechanism is controlled via **MOSFET**, directly linked to ESP32 logic.  

- **Sensors**  
  - **MLX90640 thermal IR sensor** (32×24 pixels @ 16 Hz).  
  - Mounted on the blaster itself for direct sightline tracking.  
  - Communicates with ESP32 over **I2C at 400 kHz**.  
  - **Target detection pipeline** includes:  
    - Weighted-centroid bounding box calculation  
    - Hot-pixel filtering  
    - Persistence logic to handle occlusions  

---

## 💻 Software Architecture

- **Development**  
  - Written in **C** using the Arduino IDE toolchain.  
  - Runs on **ESP32** using **dual-core FreeRTOS**.  

- **Multithreading**  
  - Two pinned tasks:  
    - Core 1 → Sensor acquisition + target detection  
    - Core 0 → Motion control + actuation  
  - Task synchronization achieved through **mutexes**.  

- **Motor Control**  
  - Stepper drivers actuated using **PWM step pulses**.  
  - **PID control** implemented with:  
    - Tuned Kp/Ki/Kd values  
    - Derivative damping  
    - Adaptive dead-zone scaling to suppress jitter near lock-on  

- **Targeting Logic**  
  - Defined **fire-box safety zone** ensures safe operation.  
  - If no target detected → system enters **“wander mode”**, scanning FOV until reacquisition.  
  - Fully parameterized configuration:  
    - Field of view  
    - Detection thresholds  
    - PID gains  
    - Motion profiles  

- **Portability**  
  - Designed as a **general-purpose turret control framework** that can be adapted to different:  
    - Sensors  
    - Motors  
    - Hardware platforms  

---

## 🚀 Iterations & Optimization

- Initial prototype: **Arduino Mega2560**  
- Transitioned to: **Arduino R4** (for higher clock & peripherals)  
- Final platform: **ESP32** (for dual-core processing + expanded memory)  

- **Blaster modification**:  
  - Overclocked from stock **7.4 V Li-Ion** pack → hardwired **9 V supply** for faster firing performance.  

---

## ⚡️ Features Summary

- Dual-core real-time control (FreeRTOS on ESP32)  
- Thermal-vision guided auto-tracking  
- Precision stepper motor pitch/yaw actuation  
- Safety-gated fire zone  
- Modular codebase adaptable to different builds  
