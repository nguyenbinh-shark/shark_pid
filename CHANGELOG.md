# Changelog

Định dạng theo [Keep a Changelog](https://keepachangelog.com/vi/1.1.0/),
đánh phiên bản theo [Semantic Versioning](https://semver.org/lang/vi/).

## [2.0.0] — 2026-08-30

Đổi mục tiêu của thư viện: từ "bộ PID nhúng đầy đủ tính năng" thành **bản sao
số của khối `PID Controller (2DOF)` trong MATLAB/Simulink**. Mỗi trường trong
`shark_pid_cfg_t` bây giờ là đúng một ô trong Block Parameters — thứ gì khối
không có thì thư viện cũng không có nữa.

Lý do: một tính năng không mô phỏng được là một tính năng không kiểm chứng
được trước khi nạp firmware. Ở 1.x, mô hình Simulink và code C chỉ trùng khít
ở những nhịp không bão hoà; đúng lúc cơ cấu chạm biên — lúc nguy hiểm nhất —
hai bên tách ra.

### Thay đổi phá vỡ tương thích

- **Bỏ feedforward**: `cfg.kf`, `cfg.kf_dot`, `pid.ff_term`,
  `shark_pid_update_ff()`, `SharkPID::updateFF()`, `SharkPID::ffTerm()`.
  Khối PID không có ô feedforward; trong Simulink bạn nối một khối `Sum` sau
  nó, và ở tầng gọi cũng chỉ là một phép cộng —
  xem [`extras/plain_c_1khz`](extras/plain_c_1khz/main.c) làm mẫu, kèm bẫy
  "phải chừa headroom cho mạch chống windup".
- **Bỏ vùng chết**: `cfg.deadband`, `SHARK_PID_F_DEADBAND_HOLD`.
- **Bỏ biến tốc độ tích phân**: `cfg.ci_a`, `cfg.ci_b`.
- **Bỏ lọc ngõ ra**: `cfg.out_tau`.
- **Bỏ giới hạn dốc**: `cfg.out_slew`. Cần thì viết vài dòng ở tầng gọi —
  xem `applySlew()` trong [`examples/02_Motor_Bidirectional`](examples/02_Motor_Bidirectional).
- **Bỏ phát hiện kẹt cơ cấu**: `cfg.stall_time`, `cfg.stall_level`,
  `cfg.stall_eps`, `SHARK_PID_F_STALL_CUTOFF`, `SHARK_PID_STALLED`,
  `SharkPID::isStalled()`.
- **Bỏ `shark_pid_clear_status()`** và `SharkPID::clearStatus()`: sau khi
  `STALLED` biến mất thì không còn cờ nào chốt lại, mọi cờ đều tính lại mỗi
  chu kỳ nên không có gì để xoá.
- **`cfg.d_tau` → `cfg.n`**: bộ lọc khâu D khai bằng **Filter coefficient N**
  (rad/giây) đúng như ô của khối, thay cho hằng số thời gian. Quy đổi:
  `n = 1 / d_tau` — mặc định `d_tau = 0.01` thành `n = 100`.
- **`cfg.kt` → `cfg.kb`**: đổi tên cho khớp ô `Back-calculation coefficient
  (Kb)`. Ở khối, `Kt` là hệ số của tracking mode — một thứ khác hẳn.
- **`pid.i_term` đổi ngữ nghĩa**: giờ là NGÕ RA của bộ tích phân trong chu kỳ
  vừa rồi (đồng bộ với `p_term`/`d_term`), còn trạng thái nằm ở `pid.i_state`.
  Với Backward Euler hai giá trị bằng nhau; với hình thang thì lệch nửa bước.
- Giá trị số của `SHARK_PID_BAD_INPUT` và `SHARK_PID_BAD_DT` đổi, do
  `SHARK_PID_STALLED` bị rút khỏi enum. Dùng tên hằng thì không ảnh hưởng.
- Bật cả `CLAMP_I` lẫn `BACKCALC_I` thì **clamping thắng** (trước đây chạy cả
  hai). Khối PID cũng chỉ chọn được một phương pháp.

### Sửa: hai chỗ 1.x không nhân bản được bằng khối PID

- **Chống windup lúc bão hoà.** Mạch anti-windup bây giờ quyết định trên đúng
  tín hiệu `preSat` mà khối nhìn — `P + TRẠNG THÁI khâu I + D`, tức chưa cộng
  đóng góp của nhịp hiện tại. 1.x tính `u_raw` của nhịp hiện tại rồi hoàn tác
  nguyên bước tích phân, nên lệch `0.01`…`0.08` ở mỗi nhịp chạm biên.
- **Kẹp `i_min`/`i_max` với I hình thang.** Khâu I tách thành cặp
  TRẠNG THÁI / NGÕ RA và kẹp riêng từng cái, đúng như khối
  `Discrete-Time Integrator`. 1.x kẹp một bộ tích luỹ duy nhất nên lệch
  `3.7e-4`.

Hệ quả: `extras/Test_Shark_PID/verify_shark_vs_pid2` chạy mô phỏng qua S-Function **ĐẠT ở mọi cấu hình**, gồm cả các cấu hình bão hoà và kẹp khâu I.

### Thêm

- `test/test_shark_pid.c` có nhóm test số 0: đối chiếu lõi C với phương trình
  sai phân của khối trên 15 cấu hình, chạy ngay trên máy tính. Sàn làm tròn đo
  được của `float` là ~4e-6, ngưỡng đặt ở 5e-5.
- `SharkPID::setFilterN()`, `SharkPID::hadBadDt()`.

### Bỏ luôn phụ thuộc

- Lõi không còn `#include <math.h>` — chỉ so sánh và bốn phép toán, nên không
  cần link `-lm`.

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
- Hai sketch Arduino trong `examples/` (lò sấy một chiều, động cơ DC hai chiều)
  và một ví dụ C thuần trong `extras/plain_c_1khz` (khớp robot 1 kHz với
  feedforward bù trọng lực, chạy được ngay trên PC).

### Kiểm chứng

- `test/test_shark_pid.c`: bộ test chạy trên máy tính, không cần vi điều khiển
  và không phụ thuộc thư viện test nào. 11 nhóm test bao gồm độc lập tần số lấy
  mẫu, chống windup, derivative kick, vùng chết, NaN/Inf, giới hạn dốc, đổi hệ
  số không giật, phát hiện kẹt, feedforward, bất biến API và bẫy `b < 1`.
- CI chạy bộ test với `gcc` và `clang` (`-Werror`), rồi chạy lại dưới
  AddressSanitizer và UBSan.
- CI cross-compile cho Cortex-M0 và Cortex-M4 với đầy đủ cảnh báo nghiêm ngặt.
- CI chạy Arduino Lint ở chế độ `library-manager: submit` và biên dịch sketch
  cho Arduino Uno.
- `extras/Test_Shark_PID`: S-function MEX cho Simulink gọi thẳng `src/shark_pid.c`
  rồi đặt song song khối `PID Controller (2DOF)` thật. Đối chiếu ĐẠT ở cả bão hoà
  với clamping lẫn back-calculation, `i_min/i_max` kiểu hình thang, Backward
  Euler, `c = 1` và `Ts = 20 ms`; sai lệch còn lại ~4e-7, đúng sàn của `float`.
