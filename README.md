# 🏥 ESP32-S3 Health Watch (Senior Edition - Debug Mode)

Dự án này là phiên bản **"Trâu bò hóa" (Robustness)** dành riêng cho việc test trên breadboard (board trắng). Code được thiết kế để chịu lỗi (Fault-tolerant), nghĩa là:
*   Rút dây cảm biến: Không crash.
*   Nhiễu nguồn: Không reset.
*   Màn hình: Luôn luôn lên.

---

## 🛠️ 1. Chuẩn Bị Phần Cứng (Lắp Mạch)

### Sơ đồ chân (Pinout) - Dual Bus I2C

| Linh Kiện | Chân | Chân ESP32-S3 (Super Mini) | Ghi Chú |
| :--- | :--- | :--- | :--- |
| **MAX30102** | SDA | **GPIO 10** (Bus 0) | Bus riêng cho PPG |
| | SCL | **GPIO 9** (Bus 0) | Bus riêng cho PPG |
| **MPU6050** | SDA | **GPIO 11** (Bus 1) | Bus phụ |
| **MAX30205**| SCL | **GPIO 12** (Bus 1) | Bus phụ |
| **Màn Hình** | MOSI | **GPIO 6** | |
| (ST7789) | SCK | **GPIO 4** | |
| | CS | **GPIO 7** | |
| | DC | **GPIO 8** | |
| | RST | **GPIO 5** | |
| | VCC | **3.3V** | Cần nguồn tốt |
| **Nút Nhấn** | Btn 1| **GPIO 2** | Boot |
| | Btn 2| **GPIO 3** | |

---

## 🔍 2. Hướng Dẫn Debug (Dành cho người mới)

Sau khi nạp code (`idf.py flash monitor`), hãy nhìn vào cửa sổ terminal. Các dòng log sẽ cho bạn biết chuyện gì đang xảy ra.

### 🟢 Log Bình Thường (Hệ thống OK)
```
I (543) MAIN: Starting Health Watch - Senior Edition
I (553) MAIN: Free PSRAM: 2097152 bytes  <-- Chip nhận đủ RAM
I (700) UI: UI Initialized               <-- Màn hình ĐÃ BẬT
I (800) MAIN: Dual I2C Initialized...    <-- Đã cài đặt I2C
I (950) MAIN: All Sensors OK             <-- Cảm biến sống
```

### 🔴 Log Lỗi & Cách Sửa

**1. Lỗi: `Hardware Error Detected. Entering ROBUST MODE...`**
*   **Nghĩa là:** Chip không tìm thấy cảm biến, nhưng nó **VẪN CHẠY TIẾP** (Robust Mode). Màn hình sẽ hiện nhưng số liệu có thể đứng im.
*   **Khắc phục:**
    *   Xem dòng bên trên nó báo lỗi gì (VD: `PPG Sensor Failed`).
    *   Kiểm tra dây nối chân GPIO 10/9.
    *   Đảo dây SDA/SCL xem có cắm ngược không.

**2. Lỗi: `PPG Read Failed (x100) - Check Wiring!`**
*   **Nghĩa là:** Cảm biến lúc đầu thì nhận, nhưng đang chạy thì bị mất tín hiệu (lỏng dây).
*   **Khắc phục:** Rung lắc nhẹ dây nối xem dây nào lỏng.

**3. Lỗi: Màn hình trắng xóa hoặc đen thui**
*   **Nghĩa là:** Code chạy (thấy log "UI Initialized") nhưng màn hình không hiển thị.
*   **Khắc phục:**
    *   Kiểm tra dây **BLK** (đèn nền). Nối nó vào 3.3V.
    *   Kiểm tra chân **RST** (GPIO 5).
    *   Thử nhấn nút Reset trên mạch ESP32.

---

## ⚙️ 3. Giải Thích Code (Cho người học C)

Code này sử dụng **FreeRTOS** (Hệ điều hành thời gian thực). Dưới đây là các khái niệm chính được dùng trong `main.c`:

### 1. Task (Tác vụ)
Giống như các "tiểu trình" chạy song song.
*   `sensor_task`: Chuyên đi lấy dữ liệu (ưu tiên cao).
*   `processing_task`: Chuyên tính toán AI (ưu tiên vừa).
*   `lvgl_task`: Chuyên vẽ màn hình (ưu tiên thấp).

### 2. Queue (Hàng đợi)
Là cái "ống nước" để truyền dữ liệu giữa các Task.
*   Task Sensor đổ nước (dữ liệu) vào đầu ống.
*   Task Processing hứng nước ở cuối ống để xử lý.
*   Lệnh `xQueueSend` (gửi) và `xQueueReceive` (nhận).

### 3. Mutex (Khóa)
Vì I2C chỉ là 1 cặp dây, nếu 2 Task cùng nhảy vào dùng 1 lúc sẽ bị nhiễu.
*   Mutex giống như cái chìa khóa toilet.
*   Ông nào có chìa khóa (`xSemaphoreTake`) thì được dùng I2C.
*   Dùng xong phải trả chìa khóa (`xSemaphoreGive`) thì ông khác mới được vào.

### 4. Semaphore (Cờ hiệu)
Dùng để báo hiệu. Trong code này dùng `Hardware Timer` để báo hiệu cho Task Sensor: "Ê, dậy đi, đến giờ đọc cảm biến rồi!" (10ms một lần).

---

## 🚀 4. Nạp Code

1.  **Build:**
    ```bash
    idf.py build
    ```
2.  **Flash:** (Giữ nút BOOT, Bấm RESET, Thả BOOT)
    ```bash
    idf.py flash monitor
    ```

Chúc bạn debug vui vẻ! Code này "trâu" lắm, khó chết lắm!