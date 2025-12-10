# ESP32-S3 Wearable Health Monitor with TinyML

## Overview
This is a comprehensive firmware for an **ESP32-S3 based Smart Health Watch**. It integrates multi-sensor data acquisition, on-device signal processing, and AI inference (TinyML) to provide real-time health metrics.

The system features a polished **LVGL 8 User Interface** with smooth double-buffered rendering and a dual-core architecture to ensure data integrity.

## Key Features

### 🩺 Health Monitoring
*   **Heart Rate (BPM):** Real-time PPG analysis using MAX30102.
*   **SpO2 (%):** Blood oxygen saturation monitoring.
*   **Respiration Rate (BrPM):** Derived from PPG signal baseline modulation (AI-assisted).
*   **Stress Level:** HRV (Heart Rate Variability) analysis classified via TensorFlow Lite Micro model.
*   **Body Temperature:** Precise skin temperature measurement via MAX30205.

### 🏃 Motion & Activity
*   **Motion Detection:** Uses MPU6050 (6-axis IMU) to detect movement/shaking.
*   **Artifact Rejection:** Automatically pauses HR/SpO2 calculation during heavy motion to prevent false readings.

### 🔋 Power Management
*   **Smart Standby Mode:** Enters a low-power state (screen off, AI pause) after 10 seconds of inactivity. Wakes up automatically on finger detection, significant motion, or any button press.

### 📱 Mobile App Connection (Bluetooth)
The watch broadcasts as **"HealthWatch"** and streams real-time data via BLE.

**Optimizations for Stability and Efficiency:**
To ensure system stability and optimize power consumption, the BLE advertising and connection parameters have been carefully tuned:
*   **Advertising Interval:** Increased from 20-40ms to **500-1000ms**. This significantly reduces radio activity when no device is connected, freeing up CPU cycles.
*   **Connection Interval:** Upon connection, the watch requests a relaxed connection interval of **100-200ms**. This minimizes frequent radio communication, allowing the main processor more time for intensive tasks like AI inference.

**How to Test & Receive Data:**
1.  Download **nRF Connect for Mobile** (Android/iOS) or **LightBlue**.
2.  Open the app and scan for devices.
3.  Find and connect to **"HealthWatch"**.
4.  Locate the Service **`0x00FF`**.
5.  Tap on Characteristic **`0xFF01`** and enable **Notifications (Subscribe)**.
6.  You will receive data strings approximately every 10 seconds (after AI processing) in this compact format:
    ```
    H:<HR_BPM>,S:<SpO2_PERCENT>,T:<TEMP_C>,B:<BAT_PERCENT>
    ```
    *   **H**: Heart Rate (e.g., 72 bpm)
    *   **S**: Blood Oxygen Saturation (SpO2, e.g., 98%)
    *   **T**: Body Temperature (e.g., 28 °C)
    *   **B**: Battery Percentage (e.g., 100%)
    
    Example: `H:72,S:98,T:28,B:100`

### 🖥️ User Interface
*   **Display:** 1.47" IPS LCD (ST7789) 172x320 resolution.
*   **Library:** LVGL 8.x with **Double Buffering** (Internal DMA) for tear-free animations.
*   **Screens:**
    1.  **Splash:** System boot & Logo.
    2.  **Main:** HR, SpO2, Temperature, System Status.
    3.  **Secondary:** Respiration, Stress Level.

### ⚙️ System Architecture
*   **Dual I2C Bus:**
    *   **Bus 0 (400kHz):** Dedicated to high-speed PPG streaming.
    *   **Bus 1 (100kHz):** Handles auxiliary sensors (IMU, Temp).
*   **Dual Core Pinning:**
    *   **Core 0:** Sensor Acquisition, Wifi, System Tasks.
    *   **Core 1:** User Interface (LVGL) & Rendering (Isolated for smoothness).
*   **Optimization:** Compiler set to **-O2 (Performance)** for stable SPI throughput.

---

## Hardware Requirements
1.  **MCU:** ESP32-S3 (DevKitC or SuperMini).
2.  **Sensors:**
    *   MAX30102 (Pulse Oximeter).
    *   MPU6050 (Accelerometer/Gyro).
    *   MAX30205 (Human Body Temperature).
3.  **Display:** 1.47 inch ST7789 SPI Module (172x320).
4.  **Buttons:** 2x Push buttons.
5.  **Battery Monitoring:** LiPo Battery with a **Voltage Divider** (e.g., 100kΩ - 100kΩ) connected to ADC input.

**👉 See `wiring_guide.md` for detailed pin connections.**

---

## Installation & Build

### 1. Prerequisites
*   **ESP-IDF v5.x** installed and configured.
*   Valid hardware connections.

### 2. Configuration
The project includes a `sdkconfig.defaults` optimized for performance, Bluetooth, and LVGL display settings. It is critical to use this configuration.

**To apply these settings and build the project:**
1.  **Clean old build artifacts and configuration:**
    ```bash
    rm -rf build sdkconfig
    ```
    *(On Windows, use `del sdkconfig` instead of `rm -rf sdkconfig`)*
2.  **Reconfigure the project (applies sdkconfig.defaults):**
    ```bash
    idf.py reconfigure
    ```
3.  **Build the project:**
    ```bash
    idf.py build
    ```

**Critical Settings Applied (via `sdkconfig.defaults`):**
*   `CONFIG_COMPILER_OPTIMIZATION_PERF=y` (Release Mode).
*   `CONFIG_FREERTOS_HZ=1000`.
*   `CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=n` (Disables USB-JTAG to free GPIO 9/10).
*   `CONFIG_PARTITION_TABLE_CUSTOM=y` (Uses `partitions.csv` for 3MB App partition).
*   `CONFIG_BT_ENABLED=y` (Bluetooth Low Energy enabled).
*   `CONFIG_LV_COLOR_16_SWAP=y` (LVGL RGB565 color swap for ST7789).

### 3. Flash & Monitor
```bash
idf.py -p /dev/ttyACM0 flash monitor
```
*(Replace `/dev/ttyACM0` with your actual serial port)*

---

## Project Structure
```
├── components/          # Drivers (MAX30102, MPU6050, MAX30205)
├── main/
│   ├── algorithms.c     # Signal processing (DSP, Filters)
│   ├── ui.c             # LVGL UI Implementation (Double Buffered)
│   ├── main.c           # FreeRTOS Tasks & Logic
│   ├── config.h         # Pinmap & System Constants
│   └── tinyml_manager.cc # TensorFlow Lite Micro Inference
├── wiring_guide.md      # Hardware connection guide
└── sdkconfig.defaults   # Critical build settings
```

## Troubleshooting

### "Task watchdog got triggered" error and System Reset?
This critical error occurs when a task (especially on CPU 1 where the AI processing runs) occupies the CPU for too long without yielding or resetting the system's Watchdog Timer. This causes the system to believe the CPU is frozen, leading to an automatic restart.

**Cause:**
*   **Heavy AI Inference:** The `tinyml_predict_respiration` model's execution time (~10.3 seconds) exceeded the default Watchdog timeout (previously 10 seconds).
*   **Aggressive BLE Advertising (Contributing Factor):** The initial high-frequency BLE advertising (every 20-40ms) created numerous CPU interrupts and memory bus contention, slightly extending the AI inference time and pushing it over the Watchdog limit.

**Solution Implemented:**
1.  **Increased Watchdog Timeout:** The system's global Task Watchdog Timer (TWDT) timeout in `sdkconfig.defaults` has been increased from 10 seconds to **25 seconds**.
2.  **BLE Parameter Optimization:**
    *   **Advertising Interval:** Significantly reduced to 500-1000ms.
    *   **Connection Interval:** Requested looser connection parameters (100-200ms) upon a successful BLE connection.
These changes minimize BLE's CPU and memory footprint, allowing the AI to complete its calculations without interruption or exceeding the adjusted Watchdog timeout.

If you encounter this error, ensure you have rebuilt the project with the latest `sdkconfig.defaults` and `ble_server.c` changes.

### "No Finger" status persists?
1. Ensure the MAX30102 sensor red LED is glowing.
2. Check wiring for I2C Bus 0 (GPIO 9/10).
3. Verify `I2C_BUS_PPG_SDA/SCL` definitions in `config.h`.

### Temperature shows "--"?
1. Check I2C Bus 1 wiring (GPIO 11/12).
2. Ensure MAX30205 address is correct (default 0x48).

---

## User Manual / Testing Guide

### 1. Power On & Boot
*   Connect power (USB or Battery).
*   Screen shows "Health Monitor v2.0" splash screen.
*   After 2 seconds, it enters the **Main Dashboard**.

### 2. Basic Operation
*   **Button NEXT (GPIO 2):** Cycle Screens (Main -> Secondary -> Main).
*   **Button PREV (GPIO 3):** Cycle Screens Backwards.
*   **Sleep Mode:** Leave the watch on the table for 10 seconds. Screen turns off.
*   **Wake Up:** Pick up the watch (shake it), put your finger on the sensor, or press any button.

### 3. Taking Measurements
1.  Strap the watch to your wrist or place finger gently on the MAX30102 sensor.
2.  Wait 3-5 seconds for the signal to stabilize.
3.  **Status Indicator:**
    *   `Scanning...`: Acquiring signal.
    *   `Measuring...`: Signal good, AI analyzing (may flicker if moving).
    *   `Stable` (Green): High-quality reading locked.
    *   `No Finger` (Red): Sensor detects emptiness.

### 4. Reading the Data
*   **Main Screen:** Heart Rate, SpO2, Temperature.
*   **Secondary Screen:** Respiration Rate, Stress Level (High/Low).
*   **Mobile App:** Check "HealthWatch" via BLE for raw data logging.