# shark_pid

[![CI](https://github.com/nguyenbinh-shark/shark_pid/actions/workflows/ci.yml/badge.svg)](https://github.com/nguyenbinh-shark/shark_pid/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![C99](https://img.shields.io/badge/C-99-blue.svg)](src/shark_pid.c)

**Bộ điều khiển PID nhúng, độc lập tần số lấy mẫu.** Lõi C99 không phụ thuộc Arduino hay HAL, kèm vỏ bọc C++ cho Arduino/ESP32/PlatformIO.

> *Sample-rate independent embedded PID controller. Portable C99 core, optional C++ wrapper. Scroll down for [English](#english).*

```
1700 byte Flash  ·  0 byte RAM tĩnh  ·  C99  ·  không cấp phát động  ·  MIT
```

Viết bởi [Trần Nguyên Bình](https://github.com/nguyenbinh-shark), như phần kết cho series [Thực Chiến Lập Trình PID](https://nguyenbinh-shark.github.io/pid-series/).

---

## Vì sao lại thêm một thư viện PID nữa?

Series PID trên blog mổ xẻ hai thư viện mã nguồn mở phổ biến. Cả hai đều hay, nhưng đều có khoảng trống thật:

| | [WangHongxi2001/PID_Library](https://github.com/WangHongxi2001/PID_Library) | [SimpleFOC](https://github.com/simplefoc/Arduino-FOC) | **shark_pid** |
|---|---|---|---|
| Tính theo `dt` thật | ✗ *(cột `ControlPeriod` khai báo mà không dùng)* | ✓ | ✓ |
| Hệ số lọc theo `dt` | ✗ *(alpha cố định)* | — *(không có lọc D)* | ✓ *(khai bằng hằng số thời gian)* |
| Tích phân hình thang | ✓ | ✓ | ✓ |
| Biến tốc độ tích phân | ✓ | ✗ | ✓ |
| Chống windup | kẹp theo dấu bộ tích luỹ | kẹp biên | kẹp có điều kiện **+** back-calculation |
| Vi phân theo giá trị đo | ✓ | ✗ | ✓ *(qua trọng số `c`)* |
| P theo giá trị đo | cờ khai báo nhưng **chưa cài đặt** | ✗ | ✓ *(qua trọng số `b`)* |
| Feedforward | ✗ | ✗ | ✓ *(tĩnh, vận tốc, và ngoài)* |
| Giới hạn dốc `du/dt` | ✗ | ✓ | ✓ |
| Ngõ ra hai chiều | ✓ | ✓ | ✓ |
| Chống NaN/Inf | ✗ | ✗ | ✓ |
| Phát hiện kẹt cơ cấu | ✓ *(chia cho `Target`, chốt vĩnh viễn)* | ✗ | ✓ *(đếm theo thời gian, xoá được)* |
| Chuyển tay→tự động không giật | ✗ | ✗ | ✓ |
| Không phụ thuộc Arduino/HAL | ✓ | ✗ | ✓ |

Chi tiết từng điểm nằm ở [`docs/so-sanh.md`](docs/so-sanh.md).

---

## Bốn nguyên tắc thiết kế

**1. `dt` là tham số tường minh.**
`Ki` có đơn vị 1/giây, `Kd` có đơn vị giây, hằng số lọc có đơn vị giây. Đổi vòng lặp từ 1 kHz xuống 50 Hz thì đáp ứng gần như không đổi — không phải dò lại hệ số.

**2. Không phụ thuộc Arduino hay HAL.**
Lõi chỉ cần `<stdint.h>` và `<math.h>`. Biên được cho STM32, ESP-IDF, AVR, hoặc chạy thẳng trên PC để mô phỏng.

**3. Tham số tự tắt.**
Đặt `d_tau = 0` là tắt lọc D. `out_slew = 0` là tắt giới hạn dốc. `deadband = 0` là tắt vùng chết. Không có cảnh "đặt tham số rồi mà quên bật cờ nên chẳng thấy gì xảy ra". Bitmask chỉ còn giữ những **lựa chọn hành vi** thật sự.

**4. Không bao giờ trả về NaN, không bao giờ chia cho 0.**
Một mẫu ADC hỏng lọt vào bộ tích phân sẽ đầu độc nó vĩnh viễn. shark_pid chặn ở cửa và giữ nguyên lệnh cũ.

---

## Cài đặt

**Arduino IDE** — Sketch → Include Library → Add .ZIP Library, trỏ vào repo này.

**PlatformIO** — thêm vào `platformio.ini`:
```ini
lib_deps = https://github.com/nguyenbinh-shark/shark_pid.git
```

**C thuần / STM32 / ESP-IDF** — chép `src/shark_pid.c` và `src/shark_pid.h` vào dự án.

---

## Dùng nhanh

### Arduino / ESP32

```cpp
#include <SharkPID.hpp>

SharkPID pid(8.0f, 0.6f, 12.0f, 0.0f, 255.0f);   // Kp, Ki, Kd, min, max

void setup() {
  pid.cfg().d_tau    = 0.30f;   // lọc khâu D, hằng số thời gian 300 ms
  pid.cfg().deadband = 0.3f;    // ±0.3 đơn vị thì thôi nhấp nhô
}

void loop() {
  float u = pid.update(60.0f, readSensor());   // dt tự đo bằng micros()
  analogWrite(PWM_PIN, (int)u);
}
```

### C thuần, chu kỳ cố định

```c
#include "shark_pid.h"

static shark_pid_t pid;

void init(void) {
    shark_pid_cfg_t cfg;
    shark_pid_cfg_default(&cfg);
    cfg.kp = 4.0f;  cfg.ki = 1.5f;  cfg.kd = 0.1f;
    cfg.out_min = -100.0f;  cfg.out_max = 100.0f;
    cfg.i_min   =  -60.0f;  cfg.i_max   =  60.0f;
    shark_pid_init(&pid, &cfg);
}

/* Trong ngắt timer 1 kHz */
float step(float target, float measured) {
    return shark_pid_update(&pid, target, measured, 0.001f);
}
```

### Feedforward tính từ ngoài

```c
/* Bù trọng lực cánh tay: mô-men giữ phụ thuộc góc, không suy ra được từ setpoint */
float gravity = HOLD_TORQUE * cosf(angle_rad);
float u = shark_pid_update_ff(&pid, target, angle_deg, dt, gravity);
```

---

## Tham số

### Hệ số cơ bản

| Trường | Đơn vị | Ý nghĩa |
|---|---|---|
| `kp` | — | Hệ số tỉ lệ |
| `ki` | 1/giây | Hệ số tích phân. `dt` được nhân bên trong |
| `kd` | giây | Hệ số vi phân. `dt` được chia bên trong |

### Setpoint weighting — PID hai bậc tự do

Hai hệ số này thay thế cho cả cặp cờ *Derivative-on-Measurement* và *Proportional-on-Measurement* của các thư viện khác:

$$u = K_p\,(b\cdot r - y) \;+\; K_i\!\!\int\!(r-y)\,dt \;+\; K_d\,\frac{d(c\cdot r - y)}{dt}$$

| Trường | Đặt | Kết quả |
|---|---|---|
| `c` | `0` *(mặc định)* | D chỉ nhìn giá trị đo → **triệt tiêu hoàn toàn** derivative kick |
| `c` | `1` | D theo sai số, kiểu cổ điển |
| `b` | `1` *(mặc định)* | P bám setpoint bình thường, đáp ứng nhanh nhất |
| `b` | `<1` | Giảm vọt lố khi đổi setpoint, đổi lại đáp ứng chậm hơn |

> **Bẫy của `b < 1` mà hầu như không tài liệu nào nhắc.** Ở trạng thái xác lập ($y = r$), khâu P xuất ra $-K_p\,r\,(1-b)$, nên **khâu I buộc phải bù đúng $K_p\,r\,(1-b)$** — cộng thêm tải tĩnh. Với `kp=4`, `r=45`, `b=0.7` thì riêng phần này đã là **54 đơn vị**. Nếu `i_max` nhỏ hơn con số đó, hệ sẽ có sai số xác lập vĩnh viễn mà nhìn hệ số PID không ra bệnh. Hạ `b` thì phải nới `i_max` theo.

### Feedforward

| Trường | Ý nghĩa |
|---|---|
| `kf` | Bù tĩnh: `kf * setpoint` |
| `kf_dot` | Bù vận tốc: `kf_dot * d(setpoint)/dt`. Chỉ có tác dụng khi setpoint đang chạy |
| *(đối số)* | `shark_pid_update_ff()` nhận thêm lượng bù tính từ ngoài |

Feedforward gánh tải tĩnh để khâu I được rảnh tay. Trong [ví dụ C thuần](extras/plain_c_1khz/main.c), FF giữ đúng 12.73 = 18·cos45° nên I về 0; tắt FF thì chính khâu I phải è cổ giữ con số đó.

### Bộ lọc — khai báo bằng hằng số thời gian

| Trường | Đơn vị | Ghi chú |
|---|---|---|
| `d_tau` | giây | Lọc khâu D. Gần như luôn cần. Mặc định 10 ms |
| `out_tau` | giây | Lọc ngõ ra. **Mặc định tắt** |

Hệ số lọc suy ra từ `dt` mỗi chu kỳ: $\alpha = dt/(\tau + dt)$. Đây là lý do đổi tần số vòng lặp không làm trôi đặc tính lọc — khác hẳn kiểu ghi cứng `alpha = 0.25`.

> **`out_tau` mặc định tắt là có chủ ý.** Lọc ngõ ra nằm *bên trong* vòng kín nên nó ăn vào biên độ pha, tức là làm hệ kém ổn định hơn. Nó bảo vệ mạch công suất, nhưng không miễn phí. Lọc khâu D thì khác — chỉ lọc một nhánh nên an toàn hơn nhiều.

### Chống windup

| Cờ | Cơ chế |
|---|---|
| `SHARK_PID_F_CLAMP_I` | Bão hoà thì hoàn tác bước tích phân vừa rồi. Đơn giản, hiệu quả — **nên dùng mặc định** |
| `SHARK_PID_F_BACKCALC_I` | Kéo bộ tích phân về tỉ lệ với lượng bị cắt, theo hệ số `kt`. Mượt hơn vì không bật/tắt đột ngột |

`kt` càng lớn thì chống windup càng mạnh (đo được là đơn điệu). Khởi điểm `kt ≈ ki/kp` rồi tăng dần đến khi vọt lố hết cải thiện.

Điều kiện kẹp dựa trên **dấu của lượng bị cắt ở ngõ ra**, không dựa vào dấu của bộ tích luỹ. Khác biệt này quan trọng: bộ tích luỹ bằng 0 lúc khởi động, nên cách kia mất tác dụng đúng vào lúc cần nhất.

### Vùng chết, giới hạn dốc, phát hiện kẹt

| Trường | Ghi chú |
|---|---|
| `deadband` | `|e|` nhỏ hơn ngưỡng thì P và I ngừng tác động. **Khâu D vẫn chạy** — vật đang trôi thì vẫn phải hãm |
| `out_slew` | `|du/dt|` tối đa. Chống sốc dòng và va đập hộp số |
| `stall_time` / `stall_level` / `stall_eps` | Ra lệnh trên `stall_level` công suất mà cơ cấu chuyển động chậm hơn `stall_eps` liên tục `stall_time` giây → báo kẹt |

Phát hiện kẹt đếm bằng **thời gian** chứ không bằng số vòng lặp, dùng trị tuyệt đối nên bắt được cả chiều quay âm, và xoá được bằng `shark_pid_clear_status()`. Mặc định chỉ **báo trạng thái**; bật `SHARK_PID_F_STALL_CUTOFF` nếu muốn tự ngắt công suất.

---

## Chuyển tay → tự động không giật

```c
shark_pid_preload(&pid, current_manual_output);   /* nạp trước bộ tích phân */
```

Ngoài ra, `i_term` lưu sẵn $K_i\!\int\!e$ (đã nhân `Ki`), nên **đổi hệ số lúc đang chạy không làm ngõ ra nhảy bậc**: hệ số mới chỉ ảnh hưởng các bước sau, không hồi tố lên lịch sử đã tích luỹ. Cách cài đặt ngây thơ `i = Ki * tổng(e)` thì nhân lại toàn bộ lịch sử bằng hệ số mới.

---

## Kết quả kiểm chứng

Đo trên mô phỏng đối chiếu (lò sấy bậc nhất, cơ cấu bão hoà 0–100%, mục tiêu 60 °C):

| Hạng mục | Kết quả |
|---|---|
| **Độc lập tần số lấy mẫu** | Đổi 1 kHz → 50 Hz *(gấp 20 lần)*, đỉnh lệch **0.03 °C**. Thiết kế không có `dt` lệch **2.51 °C** — gấp **88 lần** |
| **Chống windup (kẹp)** | Vọt lố **11.21% → 7.18%** |
| **Chống windup (back-calc)** | `kt` = 0.1 / 0.33 / 1 / 3 / 10 → **10.54 / 9.54 / 8.29 / 7.49 / 7.26%** — đơn điệu |
| **Derivative kick** | `c = 0` → đúng **0.00**. `c = 1` → +15000 |
| **Xung D giả ở vùng chết** | Nhỏ hơn **10.5 lần** so với cách zero hoá sai số rồi vi phân |
| **Đổi `Ki` gấp 4 lúc đang chạy** | Nhảy **0.30** *(đúng một bước tích phân)*. Dạng ngây thơ nhảy **45.0** — gấp 150 lần |
| **Miễn nhiễm NaN/Inf** | Giữ nguyên lệnh cũ, bật cờ `BAD_INPUT`, chu kỳ sau chạy lại bình thường |
| **Giới hạn dốc** | Không bước nào vượt trần `du/dt` |
| **Phát hiện kẹt** | Bắt được cả chiều dương lẫn chiều âm |
| **Feedforward** | Thời gian lên $t_{90}$: **8.29 s → 4.55 s** |

**Cách kiểm chứng.** Mọi con số trong bảng trên đều do [`test/test_shark_pid.c`](test/test_shark_pid.c) đo bằng chính mã C, không phải bằng mô hình xấp xỉ. Bộ test chạy trên máy tính, không cần vi điều khiển và không cần thư viện test bên ngoài.

CI chạy mỗi lần push, gồm bốn phần:

- **Test** với cả `gcc` lẫn `clang`, `-Werror`, rồi chạy lại dưới **AddressSanitizer + UBSan**.
- **Cross-compile** cho Cortex-M0 và Cortex-M4 với đầy đủ `-Wall -Wextra -Wpedantic -Wshadow -Wstrict-prototypes -Wmissing-prototypes -Wdouble-promotion -Wundef -Wcast-align -Werror`, kèm báo cáo kích thước.
- **Arduino Lint** ở chế độ `library-manager: submit` + `compliance: strict`.
- **Biên dịch sketch** cho Arduino Uno.

---

## Ví dụ

| Thư mục | Nội dung |
|---|---|
| [`examples/01_Heater_Basic`](examples/01_Heater_Basic) | Lò sấy một chiều: vùng chết, lọc D, biến tốc độ tích phân |
| [`examples/02_Motor_Bidirectional`](examples/02_Motor_Bidirectional) | Động cơ DC hai chiều qua cầu H: giới hạn dốc, phát hiện kẹt, ngõ ra âm |
| [`extras/plain_c_1khz`](extras/plain_c_1khz) | C thuần, khớp robot 1 kHz với feedforward bù trọng lực. Biên dịch chạy được ngay trên PC |

> `extras/` là thư mục chuẩn của Arduino cho những thứ IDE không dùng tới — ví dụ C thuần nằm ở đó nên `examples/` chỉ còn sketch `.ino` đúng đặc tả.

## Chạy test

```sh
make -C test test      # biên dịch và chạy toàn bộ, không cần vi điều khiển
make -C test asan      # chạy lại dưới AddressSanitizer + UBSan
```

---

## Đọc thêm

Series **Thực Chiến Lập Trình PID** giải thích từng kỹ thuật trong thư viện này:

1. [Từ công thức đến code](https://nguyenbinh-shark.github.io/posts/2026/08/pid-code-1-tu-cong-thuc-den-code/)
2. [5 căn bệnh của PID đầu tay](https://nguyenbinh-shark.github.io/posts/2026/08/pid-code-2-cac-benh-pid-dau-tay/) — bão hoà, windup, nhiễu khâu D, loạn nhịp `dt`
3. [Tinh chỉnh I và D](https://nguyenbinh-shark.github.io/posts/2026/08/pid-code-3-tinh-chinh-i-va-d/) — tích phân hình thang, biến tốc độ, vi phân theo giá trị đo
4. [Đóng gói thư viện](https://nguyenbinh-shark.github.io/posts/2026/08/pid-code-4-dong-goi-thu-vien/) — kiến trúc class và cờ tính năng
5. [Đọc hiểu PID của người đi làm](https://nguyenbinh-shark.github.io/posts/2026/08/pid-code-5-doc-pid-nguoi-di-lam/) — mổ xẻ SimpleFOC

---

## Giấy phép

MIT. Xem [LICENSE](LICENSE).

---
---

<a name="english"></a>

# shark_pid (English)

**Sample-rate independent embedded PID controller.** Portable C99 core with no Arduino or HAL dependency, plus a thin C++ wrapper for Arduino/ESP32/PlatformIO.

```
1700 bytes Flash  ·  0 bytes static RAM  ·  C99  ·  no dynamic allocation  ·  MIT
```

## Design principles

1. **`dt` is an explicit argument.** `Ki` is in 1/second, `Kd` in seconds, filter constants in seconds. Move from 1 kHz to 50 Hz and the response barely changes — no re-tuning.
2. **No Arduino, no HAL.** The core needs only `<stdint.h>` and `<math.h>`. Builds for STM32, ESP-IDF, AVR, or runs on a PC for simulation.
3. **Parameters self-disable.** `d_tau = 0` disables the derivative filter, `out_slew = 0` disables slew limiting, `deadband = 0` disables the deadband. The flag bitmask carries only genuine *behavioural choices*.
4. **Never returns NaN, never divides by zero.** One bad ADC sample poisons a naive integrator forever; shark_pid rejects it at the door and holds the previous command.

## What it has that the libraries it learned from don't

- Real `dt` in every term, and filter coefficients derived from `dt` rather than hard-coded
- Setpoint weighting (`b`, `c`) — a single mechanism that subsumes both *derivative-on-measurement* and *proportional-on-measurement*
- Feedforward: static, velocity, and an externally computed term via `shark_pid_update_ff()`
- Back-calculation anti-windup alongside conditional-integration clamping, keyed on **output** saturation rather than on the accumulator's sign
- NaN/Inf rejection, time-based recoverable stall detection, bumpless manual→auto transfer

## Quick start

```cpp
#include <SharkPID.hpp>
SharkPID pid(8.0f, 0.6f, 12.0f, 0.0f, 255.0f);   // Kp, Ki, Kd, min, max
float u = pid.update(setpoint, measurement);      // dt measured via micros()
```

```c
#include "shark_pid.h"
shark_pid_cfg_t cfg;  shark_pid_cfg_default(&cfg);
cfg.kp = 4.0f;  cfg.ki = 1.5f;  cfg.kd = 0.1f;
shark_pid_init(&pid, &cfg);
float u = shark_pid_update(&pid, setpoint, measurement, 0.001f);
```

## Gotcha worth knowing: `b < 1`

At steady state the proportional term outputs $-K_p\,r\,(1-b)$, so **the integrator must supply exactly $K_p\,r\,(1-b)$** on top of the static load. With `kp=4`, `r=45`, `b=0.7` that alone is **54 units** — if `i_max` is below it you get a permanent steady-state error that looks nothing like a gain problem. Lower `b`, raise `i_max`.

## Verification

`test/test_shark_pid.c` is a dependency-free host test suite — no microcontroller, no test framework. Run it with `make -C test test`.

CI on every push: the suite under both `gcc` and `clang` with `-Werror`, then again under AddressSanitizer + UBSan; cross-compilation for Cortex-M0 and Cortex-M4 with the full strict warning set and `-Werror`; Arduino Lint in `library-manager: submit` / `compliance: strict` mode; and sketch compilation for Arduino Uno.

## License

MIT — see [LICENSE](LICENSE).
