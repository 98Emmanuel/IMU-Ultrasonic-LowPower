# 💤 Intelligent Low-Power IMU Activation System (STM32)

## 📌 Overview

This project implements an **event-driven, low-power embedded system** using an STM32 microcontroller. The system keeps the IMU (accelerometer + gyroscope) in sleep mode and only activates it when necessary based on distance measurements from an ultrasonic sensor.

Once activated, the IMU monitors rotational motion, detects a **90° turn**, and then returns to low-power mode.

---

## ⚙️ System Architecture

* **Ultrasonic Sensor** → Detects object proximity
* **STM32 MCU** → Controls logic and power states
* **IMU (SPI)** → Measures motion and rotation
* **UART** → Outputs debug data

---

## 🔄 Operational Flow

Idle (Low Power) → Measure Distance →
If Distance < 20 cm → Activate IMU →
Measure Rotation →
If Angle ≥ 90° → Turn Complete →
Reset System → Return to Low Power

---

## 🔌 Hardware Components

* STM32 (e.g., STM32L4 series)
* SPI-based IMU sensor
* Ultrasonic sensor (HC-SR04)
* UART interface for debugging

---

## 🧩 Key Features

### 🔋 Low Power IMU Control

The IMU is turned ON only when needed and OFF otherwise to save power.

### 📏 Distance-Based Activation

Ultrasonic sensor triggers IMU activation when an object is within **20 cm**.

### 🧭 Gyroscope-Based Turn Detection

* Angular velocity is bias-corrected
* Noise filtered using a low-pass filter
* Integrated over time to compute angle

### 🧠 Sensor Calibration

* 500 samples collected at startup
* Bias computed for both accelerometer and gyroscope

### 🎛️ Noise Filtering

* Deadband removes small noise
* Low-pass filter smooths readings

---

## 🕒 Peripherals Used

| Peripheral | Function                |
| ---------- | ----------------------- |
| SPI2       | IMU communication       |
| TIM2       | Ultrasonic echo capture |
| TIM6       | Microsecond delay       |
| UART2      | Debug output            |

---

## 🔑 Key Variables

* `distance` → Measured distance (cm)
* `angle` → Computed rotation angle
* `imu_active` → IMU state flag
* `turn_done` → Turn completion flag
* `dt` → Time step for integration

---

## 📤 Example Output

Distance: 18.25
Angle: 45.32
Angle: 89.10
Turn performed

---

## 🧪 Calibration Process

1. IMU is powered ON
2. 500 samples collected
3. Bias calculated
4. IMU powered OFF

---

## 🎯 Applications

* Autonomous robotics
* Obstacle-aware navigation
* Low-power embedded systems
* Motion-triggered sensing

---

## ⚠️ Notes

* Gyroscope-only integration may introduce drift over time
* System can be improved using sensor fusion techniques

---

## 🚀 Future Improvements

* FreeRTOS-based task scheduling
* Interrupt-driven IMU activation
* Sensor fusion (Kalman/Complementary filter)
* Wireless data logging

---

## 👨‍💻 Author

**Emmanuel Odongo**
