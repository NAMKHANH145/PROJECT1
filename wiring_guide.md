# Hardware Wiring Guide
**Project:** ESP32-S3 Wearable Health Monitor

This project uses a dual I2C bus architecture and a high-speed SPI display interface. Please follow the wiring diagram below carefully to ensure system stability.

## ⚠️ CRITICAL WARNING
**Pins GPIO 9 and GPIO 10** are used for I2C Bus 0. On the ESP32-S3, these are default pins for the USB-Serial/JTAG controller.
*   **Action Required:** You MUST disable the USB-Serial-JTAG controller in `menuconfig` or use the provided `sdkconfig.defaults` to free up these pins.
*   **Result:** You will use the UART0 (TX=43, RX=44 or similar depending on board) for monitoring, not the USB OTG port.

---

## 1. Display (ST7789 1.47" IPS LCD)
**Interface:** SPI (20MHz)
**Driver:** NV3041A / ST7789

| Display Pin | ESP32-S3 Pin | Function | Notes |
| :--- | :--- | :--- | :--- |
| **VCC** | 3.3V | Power | |
| **GND** | GND | Ground | |
| **SCL / SCK** | **GPIO 4** | SPI Clock | |
| **SDA / MOSI**| **GPIO 6** | SPI Data | |
| **RES / RST** | **GPIO 5** | Reset | |
| **DC** | **GPIO 8** | Data/Command | |
| **CS** | **GPIO 7** | Chip Select | |
| **BLK** | **GPIO 13** | Backlight | Logic 1 = ON. Moved from GPIO 1 to free up pin for Battery ADC. |

---

## 2. Sensor Bus 0: PPG (Heart Rate/SpO2)
**Sensor:** MAX30102
**Interface:** I2C (400kHz - Fast Mode)

| Sensor Pin | ESP32-S3 Pin | Notes |
| :--- | :--- | :--- |
| **VCC** | 3.3V | **Stable** 3.3V required |
| **GND** | GND | |
| **SDA** | **GPIO 10** | Requires Pull-up (2.2k - 4.7k) |
| **SCL** | **GPIO 9** | Requires Pull-up (2.2k - 4.7k) |
| **INT** | N/C | Not used in this firmware |

---

## 3. Sensor Bus 1: Auxiliary (Motion & Temp)
**Sensors:** MPU6050 (IMU) + MAX30205 (Body Temp)
**Interface:** I2C (100kHz - Standard Mode)

| Sensor Pin | ESP32-S3 Pin | Notes |
| :--- | :--- | :--- |
| **VCC** | 3.3V | |
| **GND** | GND | |
| **SDA** | **GPIO 11** | Shared Bus |
| **SCL** | **GPIO 12** | Shared Bus |

*Note: Connect both MPU6050 and MAX30205 SDA/SCL pins to this bus in parallel.*

---

## 4. User Interface (Buttons)
**Type:** Tactile Push Buttons (Active Low)

| Button | ESP32-S3 Pin | Logic |
| :--- | :--- | :--- |
| **NEXT** | **GPIO 2** | Press = GND (0), Release = 3.3V (1) |
| **PREV** | **GPIO 3** | Press = GND (0), Release = 3.3V (1) |

*Note: Internal Pull-ups are enabled in software. Connect one leg to GPIO, the other to GND.*

---

## 5. Battery Monitoring (LiPo)
**Interface:** Analog to Digital Converter (ADC)

| Component | ESP32-S3 Pin | Notes |
| :--- | :--- | :--- |
| **Battery Voltage** | **GPIO 1** | Connect via Voltage Divider |

**⚠️ CRITICAL VOLTAGE DIVIDER WARNING ⚠️**
*   LiPo battery voltage (typically 3.0V - 4.2V) **MUST NOT** be connected directly to any ESP32-S3 GPIO pin. ESP32-S3 pins are 3.3V tolerant.
*   You **MUST** use a voltage divider circuit to reduce the battery voltage to a safe range (max 3.3V) for **GPIO 1** (ADC1_CH0).
*   **Recommended Setup:** Use two 100kΩ resistors (R1 and R2).
    *   Connect `Battery +` to one end of `R1`.
    *   Connect the other end of `R1` to `GPIO 1` and also to one end of `R2`.
    *   Connect the other end of `R2` to `GND`.
*   This setup divides the battery voltage by 2, ensuring a safe input range for the ADC (e.g., 4.2V battery becomes 2.1V input).

---

## 6. Power Supply
*   **Source:** LiPo Battery (3.7V) via LDO or USB 5V -> 3.3V Regulator.
*   **Current Requirement:** The system peaks at ~200mA (WiFi + Screen + LEDs). Ensure your regulator can supply at least 500mA.