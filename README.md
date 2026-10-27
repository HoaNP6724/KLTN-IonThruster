# HƯỚNG DẪN SỬ DỤNG WEB DASHBOARD ESP32

Hệ thống điều khiển và giám sát toàn diện qua Web gồm:
1. **Module xung XY-LPWM**: Đọc thông số và cài đặt tần số (Fxxx), chu kỳ xung Duty Cycle (Dxxx).
2. **Cảm biến gió RS485 Modbus-RTU**: Đọc tốc độ gió tức thời (m/s), hiển thị đồng hồ Gauge xoay tròn, cấp gió Beaufort và đồ thị biến thiên thời gian thực.
3. **Module Rơ-le (Relay)**: Điều khiển Bật/Tắt thiết bị ngoại vi tức thời với công tắc toggle lớn và đèn LED hiển thị trạng thái.

---

## 1. KHÔNG LÀM THAY ĐỔI CÁC FILE GỐC
Hệ thống tuân thủ nghiêm ngặt yêu cầu của bạn:
- `main/main.c` ➡️ **GIỮ NGUYÊN 100%**
- `main/lpwm_backup.c` ➡️ **GIỮ NGUYÊN 100%**
- `main/wind_sensor_backup.c` ➡️ **GIỮ NGUYÊN 100%**

Các thành phần mới được tạo riêng biệt:
- `web/index.html`: Giao diện Web Dashboard cao cấp, Dark Theme Glassmorphism hiện đại, hỗ trợ cả chế độ Demo mô phỏng và kết nối ESP32 thực tế.
- `main/web_server_system.c` & `main/web_server_system.h`: Mã nguồn C backend tích hợp WiFi SoftAP, HTTP Server, xử lý Modbus-RTU cảm biến gió, UART module xung và điều khiển Relay.

---

## 2. TRẢI NGHIỆM GIAO DIỆN WEB NGAY BÂY GIỜ
Bạn có thể mở trực tiếp file:
📁 **`web/index.html`** bằng trình duyệt (Chrome, Edge, Cốc Cốc, Firefox):

1. Click đúp chuột vào file `web/index.html`.
2. Khi mở trực tiếp từ máy tính (`file:///`), trang web sẽ tự động bật **"Chế độ mô phỏng (Demo Mode)"**:
   - Bạn có thể kéo thử slider chu kỳ xung Duty Cycle, chọn các preset tần số (50Hz, 1kHz, 10kHz, 100kHz).
   - Nhấn **"Gửi Cài Đặt (Apply)"** hoặc **"Đọc thông số (Read)"** để xem phản hồi mô phỏng tại khung nhật ký terminal.
   - Quan sát kim đồng hồ tốc độ gió và đồ thị sóng biến thiên tự động cập nhật mỗi 2 giây.
   - Bấm công tắc **BẬT/TẮT Relay** để thấy hiệu ứng đèn phát sáng và phản hồi giao diện.
3. Khi ESP32 đã phát WiFi, bạn chỉ cần nhập IP `192.168.4.1` vào ô Host ở góc trên rồi bấm **"Lưu"**, tắt nút Demo là trang web sẽ lập tức giao tiếp với phần cứng thực tế.

---

## 3. SƠ ĐỒ KẾT NỐI PHẦN CỨNG ESP32

### A. Module Tạo Xung XY-LPWM (UART1)
| Chân ESP32 | Chân Module XY-LPWM | Chú thích |
| :--- | :--- | :--- |
| **GPIO 18** (TX1) | **RXD** | Gửi lệnh Fxxx, Dxxx, read |
| **GPIO 19** (RX1) | **TXD** | Nhận phản hồi DOWN/FALL |
| **GND** | **GND** | Nối chung đất |
| **5V / VIN** | **VCC** | Cấp nguồn module xung |

### B. Cảm Biến Gió RS485 qua Module MAX485 (UART2 - Half Duplex)
| Chân ESP32 | Chân MAX485 | Chú thích |
| :--- | :--- | :--- |
| **GPIO 17** (TX2) | **DI** | Driver Input |
| **GPIO 16** (RX2) | **RO** | Receiver Output |
| **GPIO 4** | **DE + RE** (nối chung) | Chân điều hướng truyền/nhận tự động |
| **GND** | **GND** | Mass chung |
| **3.3V hoặc 5V** | **VCC** | Nguồn MAX485 |
| *Không nối* | **A** | Dây Vàng (485-A cảm biến) |
| *Không nối* | **B** | Dây Xanh lá (485-B cảm biến) |

### C. Module Rơ-le (Relay)
| Chân ESP32 | Chân Relay | Chú thích |
| :--- | :--- | :--- |
| **GPIO 23** | **IN** | Tín hiệu điều khiển đóng/ngắt tải |
| **GND** | **GND** | Nối chung Mass |
| **5V / 3.3V** | **VCC** | Nguồn nuôi cuộn hút relay |

---

## 4. CÁCH KÍCH HOẠT CHẠY TRÊN ESP32 KHI SẴN SÀNG BUILD FIRMWARE

Khi bạn muốn nạp toàn bộ hệ thống Web Server vào chip ESP32, bạn chỉ cần:

### Cách 1: Gọi hàm từ `app_main`
Trong hàm `app_main(void)`, gọi hàm khởi chạy:
```c
#include "web_server_system.h"

void app_main(void) {
    web_app_system_start();
}
```

### Cách 2: Thông tin kết nối WiFi sau khi nạp ESP32
- **Tên WiFi phát từ ESP32 (SSID)**: `ESP32-KLTN-HUB`
- **Mật khẩu (Password)**: `12345678`
- **Địa chỉ truy cập Web**: `http://192.168.4.1/`

---

## 5. DANH SÁCH REST API ĐÃ ĐƯỢC LẬP TRÌNH SẴN TRÊN ESP32
- `GET /api/status`: Lấy toàn bộ trạng thái (tốc độ gió, cấp gió, trạng thái relay, cấu hình PWM hiện tại).
- `GET /api/wind`: Lấy tốc độ gió tức thời.
- `POST /api/relay`: Nhận JSON `{"state": 1}` để bật, `{"state": 0}` để tắt.
- `POST /api/pwm`: Nhận JSON `{"freq": "F1.00", "duty": "D050"}` để thiết lập xung.
- `GET /api/pwm/read`: Đọc dữ liệu trực tiếp từ module XY-LPWM.
