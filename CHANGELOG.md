# Changelog

Định dạng theo [Keep a Changelog](https://keepachangelog.com/vi/1.1.0/),
đánh phiên bản theo [Semantic Versioning](https://semver.org/lang/vi/).

## [1.0.0] — 2026-08-29

Bản đầu tiên.

### Lõi

- Lõi C99 (`shark_pid.c` / `shark_pid.h`) không phụ thuộc Arduino hay HAL.
- `dt` là tham số tường minh của `shark_pid_update()`, đơn vị giây. `Ki` tính
  theo 1/giây, `Kd` theo giây — đổi tần số vòng lặp không phải dò lại hệ số.
- Toàn bộ trạng thái nằm trong struct do người gọi cấp phát. Không `malloc`,
  không biến toàn cục, không RAM tĩnh.

### Thuật toán

- Tích phân hình thang (Tustin) hoặc hình chữ nhật.
- Setpoint weighting hai bậc tự do qua `cfg.b` và `cfg.c`, thay cho cặp cờ
  *Derivative-on-Measurement* / *Proportional-on-Measurement*.
- Chống windup hai kiểu: kẹp có điều kiện (`SHARK_PID_F_CLAMP_I`) và
  back-calculation (`SHARK_PID_F_BACKCALC_I`, hệ số `cfg.kt`). Cả hai xét dấu
  của lượng bị cắt ở **ngõ ra**, không xét dấu bộ tích luỹ.
- Biến tốc độ tích phân theo hai ngưỡng `ci_a` / `ci_b`.
- Feedforward tĩnh (`kf`), theo vận tốc setpoint (`kf_dot`), và lượng bù tính
  từ ngoài qua `shark_pid_update_ff()`.
- Lọc thông thấp khâu D và khâu ngõ ra, khai báo bằng **hằng số thời gian**
  (giây); hệ số $\alpha = dt/(\tau + dt)$ suy lại mỗi chu kỳ.
- Giới hạn dốc `du/dt` (`out_slew`).
- Vùng chết, kèm chế độ đóng băng lệnh `SHARK_PID_F_DEADBAND_HOLD`. Khâu D vẫn
  hoạt động trong vùng chết nên không sinh xung vi phân giả.

### An toàn

- Chặn NaN/Inf ở đầu vào: giữ nguyên lệnh cũ, bật cờ `SHARK_PID_BAD_INPUT`,
  bộ tích phân không bị nhiễm độc.
- Bảo vệ `dt`: giá trị âm, bằng 0, NaN hoặc lớn hơn `dt_max` đều bị thay bằng
  `dt_nominal` và bật cờ `SHARK_PID_BAD_DT`.
- Phát hiện kẹt cơ cấu đếm theo **thời gian** (`stall_time`), dùng trị tuyệt đối
  nên bắt được cả chiều quay âm, không có phép chia nào, và xoá được bằng
  `shark_pid_clear_status()`. Mặc định chỉ báo trạng thái; bật
  `SHARK_PID_F_STALL_CUTOFF` mới tự ngắt công suất.
- Ngõ ra có `out_min` và `out_max` riêng biệt, hỗ trợ cơ cấu hai chiều.
- Trần tích phân `i_min` / `i_max` tách khỏi trần ngõ ra.

### Chuyển chế độ

- `shark_pid_preload()` nạp trước bộ tích phân cho chuyển tay → tự động không giật.
- Bộ tích luỹ chứa sẵn `Ki` nên đổi hệ số lúc đang chạy không làm ngõ ra nhảy bậc.

### Vỏ bọc C++

- `SharkPID.hpp` cho Arduino / ESP32 / PlatformIO.
- Overload `update(setpoint, measurement)` tự đo `dt` bằng `micros()`, dùng phép
  trừ không dấu nên đúng cả khi bộ đếm 32-bit tràn.
- `operator()` để gọi theo kiểu functor.

### Tài liệu và ví dụ

- README song ngữ Việt / Anh.
- [`docs/so-sanh.md`](docs/so-sanh.md): đối chiếu chi tiết với
  WangHongxi2001/PID_Library và SimpleFOC, kèm bảng tra cờ cũ → tham số mới và
  công thức quy đổi hệ số.
- Ba ví dụ: lò sấy một chiều, động cơ DC hai chiều, và khớp robot 1 kHz viết
  bằng C thuần chạy được ngay trên PC.

### Còn nợ

- Bộ test C chạy trực tiếp trên PC. Bản 1.0.0 được kiểm chứng bằng biên dịch
  nghiêm ngặt (`arm-none-eabi-gcc 12.3.1`, C99, không cảnh báo) cộng với một
  bản chuyển ngữ 1:1 sang Python chạy trên mô hình đối tượng.
