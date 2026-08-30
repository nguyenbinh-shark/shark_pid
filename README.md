# shark_pid

[![CI](https://github.com/nguyenbinh-shark/shark_pid/actions/workflows/ci.yml/badge.svg)](https://github.com/nguyenbinh-shark/shark_pid/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![C99](https://img.shields.io/badge/C-99-blue.svg)](src/shark_pid.c)

**Khối `PID Controller (2DOF)` của Simulink, viết bằng C99 để chạy trên vi điều khiển.** Mỗi tham số là đúng một ô trong Block Parameters; mô hình bạn dò trong mô phỏng chính là bộ điều khiển chạy trên phần cứng.

> *The Simulink PID Controller (2DOF) block, reimplemented in portable C99 for microcontrollers. Scroll down for [English](#english).*

```
~1100 byte Flash  ·  0 byte RAM tĩnh  ·  C99  ·  không cấp phát động  ·  không cần -lm  ·  MIT
```

Viết bởi [Trần Nguyên Bình](https://github.com/nguyenbinh-shark), như phần kết cho series [Thực Chiến Lập Trình PID](https://nguyenbinh-shark.github.io/pid-series/).

---

## Vấn đề nó giải quyết

Bạn dò PID trong Simulink, đồ thị đẹp, nạp xuống STM32 — và hệ chạy khác. Không phải vì mô hình đối tượng sai, mà vì **bộ điều khiển trong mô phỏng và bộ điều khiển trong firmware là hai thứ khác nhau**: khác cách rời rạc hoá khâu I, khác cách lọc khâu D, và khác nhất là cách chống windup lúc cơ cấu chạm biên.

shark_pid xoá khoảng cách đó bằng cách đi ngược lại: thay vì thêm tính năng cho thư viện C rồi tìm cách mô phỏng, nó **chép đúng sơ đồ bên trong khối PID** và không có gì hơn.

| | |
|---|---|
| Tham số | 100% ánh xạ 1-1 với một ô trong Block Parameters |
| Sai lệch so với khối, ở cấu hình không bão hoà | `1e-15` *(double)* |
| Sai lệch so với khối, ở cấu hình **bão hoà** | `1e-15` *(double)* — chỗ mà 1.x không làm được |
| Sai lệch thực tế trên MCU | `~4e-6` — sàn làm tròn của `float`, không khử được |

Thư mục [`extras/Test_Shark_PID/`](extras/Test_Shark_PID/) có sẵn bộ đối chiếu `verify_shark_vs_pid2`: dựng mô hình Simulink gọi thẳng mã C thật qua S-Function để so sánh với khối `PID Controller (2DOF)`.

---

## Bảng ánh xạ

| `shark_pid_cfg_t` | Ô trong Block Parameters |
|---|---|
| — | `PID Controller (2DOF)`, `Form = Parallel`, `Time domain = Discrete-time` |
| `dt` *(đối số của `update`)* | `Sample time` |
| `kp`, `ki`, `kd` | `P`, `I`, `D` |
| `b`, `c` | `Setpoint weight (b)`, `Setpoint weight (c)` |
| `n > 0` | tích `Use filtered derivative`, `Filter coefficient (N)` = `n`, `Filter method = Backward Euler` |
| `n <= 0` | bỏ tích `Use filtered derivative` |
| cờ `TRAPEZOID_I` bật / tắt | `Integrator method` = `Trapezoidal` / `Backward Euler` |
| `out_min`, `out_max` | `Limit output` + `Lower/Upper saturation limit` |
| `i_min`, `i_max` | `Limit integrator` + `Lower/Upper integrator saturation limit` |
| cờ `CLAMP_I` | `Anti-windup method = clamping` |
| cờ `BACKCALC_I`, `kb` | `Anti-windup method = back-calculation`, `Kb` = `kb` |
| `dt_max`, `dt_nominal` | *(không có ô tương ứng — xem bên dưới)* |

Chạy `verify_shark_vs_pid2(...)` trong `extras/Test_Shark_PID` sẽ tự động đổ các tham số này vào mô hình Simulink để đối chiếu.

---

## Bốn nguyên tắc thiết kế

**1. Không có tính năng nào khối PID không có.**
Một tính năng không mô phỏng được là một tính năng không kiểm chứng được trước khi nạp firmware. Cần vùng chết, giới hạn dốc hay feedforward? Trong Simulink chúng là khối `Dead Zone`, `Rate Limiter`, `Sum` nối *sau* khối PID — nên ở đây chúng cũng nằm *ngoài* bộ điều khiển, ở tầng gọi. [Ví dụ 02](examples/02_Motor_Bidirectional) làm mẫu giới hạn dốc trong 8 dòng.

**2. `dt` là tham số tường minh.**
`Ki` có đơn vị 1/giây, `Kd` có đơn vị giây, `N` có đơn vị rad/giây. Đổi vòng lặp từ 1 kHz xuống 50 Hz thì đáp ứng gần như không đổi — không phải dò lại hệ số. **Đây là thứ duy nhất shark_pid có mà khối PID không có**, vì `Sample time` của khối là hằng số còn ngắt timer thật thì jitter.

**3. Không phụ thuộc Arduino hay HAL.**
Lõi chỉ cần `<stdint.h>`. Không gọi hàm nào của `<math.h>` nên không phải link `-lm`. Biên được cho STM32, ESP-IDF, AVR, hoặc chạy thẳng trên PC để mô phỏng.

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
  pid.cfg().n = 3.33f;          // Filter coefficient N (~ lọc D 300 ms)
  pid.cfg().c = 0.0f;           // Setpoint weight (c): D chỉ nhìn cảm biến
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
    cfg.n  = 250.0f;                        /* Filter coefficient N */
    cfg.out_min = -100.0f;  cfg.out_max = 100.0f;
    cfg.i_min   =  -60.0f;  cfg.i_max   =  60.0f;
    shark_pid_init(&pid, &cfg);
}

/* Trong ngắt timer 1 kHz */
float step(float target, float measured) {
    return shark_pid_update(&pid, target, measured, 0.001f);
}
```

### Đối chiếu với mô hình Simulink

```matlab
addpath('extras/Test_Shark_PID');
verify_shark_vs_pid2('kp', 4, 'ki', 1.5, 'kd', 0.1, 'n', 250, 'Ts', 0.001);
```

---

## Tham số

### Hệ số cơ bản

| Trường | Ô của khối | Đơn vị |
|---|---|---|
| `kp` | `P` | — |
| `ki` | `I` | 1/giây. `dt` được nhân bên trong |
| `kd` | `D` | giây. `dt` được chia bên trong |

### Setpoint weighting — phần "2 Bậc Tự Do" (2DOF)

**Tại sao lại là "2 Bậc Tự Do" (2DOF)?** 
Một bộ PID truyền thống (1-DOF) chỉ nhận duy nhất tín hiệu Sai số ($e = r - y$), dẫn đến nhược điểm chí mạng: khi người dùng thay đổi giá trị đặt (Setpoint) một cách đột ngột, sai số nhảy vọt tức thời làm khâu P và D phản ứng thái quá, gây ra hiện tượng **vọt lố (overshoot)** và **đá vi phân (derivative kick)** làm rần hệ thống.

**PID 2DOF** giải quyết triệt để vấn đề này bằng cách nhận tách biệt Setpoint ($r$) và Measurement ($y$), cho phép nhân thêm **trọng số điểm đặt (setpoint weights - $b$ và $c$)** để kìm hãm sự hưng phấn của P và D, nhưng **không hề làm chậm đi khả năng bù nhiễu**. Đó cũng là lý do thư viện yêu cầu truyền tham số tách biệt: `shark_pid_update(&pid, setpoint, measurement, dt);`.

Phương trình thời gian liên tục lý tưởng:
$$u = K_p\,(b\cdot r - y) \;+\; K_i\!\!\int\!(r-y)\,dt \;+\; \text{Dbranch}(c\cdot r - y)$$

| Trường | Đặt | Kết quả |
|---|---|---|
| `c` | `0` *(mặc định)* | D chỉ nhìn giá trị đo → **triệt tiêu hoàn toàn** derivative kick |
| `c` | `1` | D theo sai số, kiểu cổ điển |
| `b` | `1` *(mặc định)* | P bám setpoint bình thường, đáp ứng nhanh nhất |
| `b` | `<1` | Giảm vọt lố khi đổi setpoint, đổi lại đáp ứng chậm hơn |

> **Bẫy của `b < 1` mà hầu như không tài liệu nào nhắc.** Ở trạng thái xác lập ($y = r$), khâu P xuất ra $-K_p\,r\,(1-b)$, nên **khâu I buộc phải bù đúng $K_p\,r\,(1-b)$** — cộng thêm tải tĩnh. Với `kp=4`, `r=45`, `b=0.7` thì riêng phần này đã là **54 đơn vị**. Nếu `i_max` nhỏ hơn con số đó, hệ sẽ có sai số xác lập vĩnh viễn mà nhìn hệ số PID không ra bệnh. Hạ `b` thì phải nới `i_max` theo.

### Bộ lọc khâu D — khai bằng `N`, đúng như khối

| Trường | Đơn vị | Ghi chú |
|---|---|---|
| `n` | rad/giây | Ô `Filter coefficient (N)`. Mặc định 100. `n <= 0` = bỏ lọc |

Hằng số thời gian tương đương là `1/N` giây: `n = 100` ↔ 10 ms. Hệ số rời rạc suy lại từ `dt` mỗi chu kỳ nên đổi tần số vòng lặp không làm trôi tần số cắt — khác hẳn kiểu ghi cứng `alpha = 0.25`.

> Ở 1.x trường này là `d_tau` (hằng số thời gian) và phải quy đổi `N = 1/d_tau` khi điền vào hộp thoại. Từ 2.0 khai thẳng bằng `N` cho khỏi có chỗ mà sai.

### Chống windup

| Cờ | Cơ chế |
|---|---|
| `SHARK_PID_F_CLAMP_I` | `Anti-windup method = clamping`. Ngắt tích phân khi lượng vượt biên và `preInt` cùng dấu. **Nên dùng mặc định** |
| `SHARK_PID_F_BACKCALC_I` | `Anti-windup method = back-calculation`. Kéo bộ tích phân về theo lượng bị cắt, hệ số `kb`. Mượt hơn vì không bật/tắt đột ngột |

Khối chỉ chọn được **một** phương pháp; bật cả hai cờ thì lõi C cũng ưu tiên clamping, đúng như vậy.

`kb` càng lớn thì chống windup càng mạnh (đo được là đơn điệu). Khởi điểm `kb ≈ ki/kp` rồi tăng dần đến khi vọt lố hết cải thiện.

**Chi tiết đáng đọc.** Mạch chống windup nhìn vào tín hiệu `preSat = P + TRẠNG THÁI khâu I + D` — tức **chưa cộng đóng góp của nhịp hiện tại**. Simulink buộc phải làm vậy để cắt vòng đại số, vì Backward Euler và Trapezoidal đều có truyền thẳng. Cài đặt trực quan hơn là "tính `u_raw` của nhịp này rồi hoàn tác bước tích phân" — đó chính là cách shark_pid 1.x làm, và cũng chính là lý do 1.x lệch với khối đúng những nhịp chạm biên. 2.0 theo đúng khối.

### Bảo vệ nhịp lấy mẫu

| Trường | Ghi chú |
|---|---|
| `dt_max` | `dt` lớn hơn ngưỡng này coi là bất thường |
| `dt_nominal` | `dt` thay thế khi bất thường. Cũng chính là `Sample time` của khối |

Đây là hai trường **duy nhất** không có ô tương ứng, vì khối PID trong Simulink có `Ts` hằng số còn vi điều khiển thực tế thì chu kỳ ngắt có thể bị jitter.

---

## Những gì cố tình KHÔNG có

Khối PID không có, nên thư viện cũng không. Cần thì thêm ở tầng gọi — đúng như trong Simulink bạn nối thêm khối:

| Cần gì | Trong Simulink | Trong code |
|---|---|---|
| Feedforward | khối `Sum` sau khối PID | `u = shark_pid_update(...) + ff;` — [xem ví dụ](extras/plain_c_1khz/main.c), kèm bẫy phải chừa headroom cho chống windup |
| Giới hạn dốc `du/dt` | `Rate Limiter` | `applySlew()` trong [ví dụ 02](examples/02_Motor_Bidirectional), 8 dòng |
| Vùng chết | `Dead Zone` | một câu `if` quanh sai số |
| Phát hiện kẹt cơ cấu | logic riêng | đếm thời gian ở tầng ứng dụng, dùng `pid.isSaturated()` |
| Lọc ngõ ra | `Discrete Filter` | lọc thông thấp sau lệnh |
| Biến tốc độ tích phân | *(không có)* | nếu thật sự cần thì hệ đang thiếu một khâu khác |

---

## Chuyển tay → tự động không giật

```c
shark_pid_preload(&pid, current_manual_output);   /* nạp trước bộ tích phân */
```

Tương đương điền `Integrator Initial condition` trong tab Initialization của khối, cộng với việc ép khâu D bằng 0 ở nhịp đầu.

Ngoài ra, `i_state` lưu sẵn $K_i\!\int\!e$ (đã nhân `Ki`), nên **đổi hệ số lúc đang chạy không làm ngõ ra nhảy bậc**: hệ số mới chỉ ảnh hưởng các bước sau, không hồi tố lên lịch sử đã tích luỹ. Cách cài đặt ngây thơ `i = Ki * tổng(e)` thì nhân lại toàn bộ lịch sử bằng hệ số mới.

---

## Kết quả kiểm chứng

| Hạng mục | Kết quả |
|---|---|
| **Khớp khối PID (2DOF)** | 15/15 cấu hình đạt `1e-15` — gồm 4 cấu hình bão hoà và 2 cấu hình kẹp khâu I |
| **Sàn làm tròn `float`** | `~4e-6` trên toàn bộ 15 cấu hình |
| **Độc lập tần số lấy mẫu** | Đổi 1 kHz → 50 Hz *(gấp 20 lần)*, đỉnh lệch **0.002 °C**. Thiết kế không có `dt` lệch **2.51 °C** — gấp hơn **1000 lần** |
| **Chống windup (clamping)** | Vọt lố **11.15% → 7.20%** |
| **Chống windup (back-calc)** | `kb` = 0.1 / 0.33 / 1 / 3 / 10 → **10.50 / 9.52 / 8.31 / 7.51 / 7.28%** — đơn điệu |
| **Derivative kick** | `c = 0` → đúng **0.00**. `c = 1` → +15000 |
| **Bộ lọc `N`** | Vào dốc đều, D xác lập đúng $-K_d\,\dot y$; `N` nhỏ lọc mạnh hơn hẳn |
| **Đổi `Ki` gấp 4 lúc đang chạy** | Nhảy **0.19** *(dưới một bước tích phân)*. Dạng ngây thơ nhảy **44.9** — gấp 230 lần |
| **Miễn nhiễm NaN/Inf** | Giữ nguyên lệnh cũ, bật cờ `BAD_INPUT`, chu kỳ sau chạy lại bình thường |
| **Bẫy `b < 1`** | Khâu I gánh thêm đúng $K_p\,r\,(1-b)$ = 72.0, khớp lý thuyết |

**Cách kiểm chứng.** Mọi con số trong bảng trên đều do [`test/test_shark_pid.c`](test/test_shark_pid.c) đo bằng chính mã C, không phải bằng mô hình xấp xỉ. Bộ test chạy trên máy tính, không cần vi điều khiển và không cần thư viện test bên ngoài. Nhóm test số 0 nhúng luôn phương trình sai phân chuẩn của khối PID (vốn đã đối chiếu `1e-15` với khối thật trong Simulink R2022b qua S-Function) nên chuyện "khớp khối" được canh gác ở mỗi lần push.

CI chạy mỗi lần push, gồm bốn phần:

- **Test** với cả `gcc` lẫn `clang`, `-Werror`, rồi chạy lại dưới **AddressSanitizer + UBSan**.
- **Cross-compile** cho Cortex-M0 và Cortex-M4 với đầy đủ `-Wall -Wextra -Wpedantic -Wshadow -Wstrict-prototypes -Wmissing-prototypes -Wdouble-promotion -Wundef -Wcast-align -Werror`, kèm báo cáo kích thước.
- **Arduino Lint** ở chế độ `library-manager: submit` + `compliance: strict`.
- **Biên dịch sketch** cho Arduino Uno.

---

## Ví dụ

| Thư mục | Nội dung |
|---|---|
| [`examples/01_Heater_Basic`](examples/01_Heater_Basic) | Lò sấy một chiều: lọc D bằng `N`, kẹp tích phân, chống windup clamping |
| [`examples/02_Motor_Bidirectional`](examples/02_Motor_Bidirectional) | Động cơ DC hai chiều qua cầu H: ngõ ra âm, back-calculation, và giới hạn dốc **để ngoài lõi** |
| [`extras/Test_Shark_PID`](extras/Test_Shark_PID) | Đối chiếu và mô phỏng với Simulink/MATLAB: S-Function MEX gọi thẳng `src/shark_pid.c`, self-test và bảng ánh xạ |

> `extras/` là thư mục chuẩn của Arduino cho những thứ IDE không dùng tới — ví dụ C thuần nằm ở đó nên `examples/` chỉ còn sketch `.ino` đúng đặc tả.

## Chạy test

```sh
make -C test test      # biên dịch và chạy toàn bộ, không cần vi điều khiển
make -C test asan      # chạy lại dưới AddressSanitizer + UBSan
```

---

## Nâng cấp từ 1.x

Xem [CHANGELOG.md](CHANGELOG.md) để có danh sách đầy đủ. Bốn chỗ hay gặp nhất:

```c
cfg.d_tau = 0.01f;   ->   cfg.n = 100.0f;        /* N = 1/d_tau */
cfg.kt    = 3.0f;    ->   cfg.kb = 3.0f;         /* đổi tên cho khớp ô Kb */
shark_pid_update_ff(&pid, r, y, dt, ff);
                     ->   shark_pid_update(&pid, r, y, dt) + ff;
pid.i_term                                       /* giờ là NGÕ RA khâu I;
                                                    trạng thái ở pid.i_state */
```

`deadband`, `ci_a`/`ci_b`, `out_tau`, `out_slew`, `stall_*` và `shark_pid_clear_status()` đã bị bỏ — bảng [Những gì cố tình KHÔNG có](#những-gì-cố-tình-không-có) chỉ chỗ thay thế cho từng món.

---

## Đọc thêm

Series **Thực Chiến Lập Trình PID** giải thích từng kỹ thuật trong thư viện này:

1. [Từ công thức đến code](https://nguyenbinh-shark.github.io/posts/2026/08/pid-code-1-tu-cong-thuc-den-code/)
2. [5 căn bệnh của PID đầu tay](https://nguyenbinh-shark.github.io/posts/2026/08/pid-code-2-cac-benh-pid-dau-tay/) — bão hoà, windup, nhiễu khâu D, loạn nhịp `dt`
3. [Tinh chỉnh I và D](https://nguyenbinh-shark.github.io/posts/2026/08/pid-code-3-tinh-chinh-i-va-d/) — tích phân hình thang, vi phân theo giá trị đo
4. [Đóng gói thư viện](https://nguyenbinh-shark.github.io/posts/2026/08/pid-code-4-dong-goi-thu-vien/) — kiến trúc class và cờ tính năng
5. [Đọc hiểu PID của người đi làm](https://nguyenbinh-shark.github.io/posts/2026/08/pid-code-5-doc-pid-nguoi-di-lam/) — mổ xẻ SimpleFOC
6. [Cấu hình khối PID Simulink "chuẩn vị" code C](https://nguyenbinh-shark.github.io/posts/2026/08/simscape-multibody-pid-simulink-sim2real/) — bài mà thư viện 2.0 này là kết luận

Còn [`docs/so-sanh.md`](docs/so-sanh.md) ghi lại thư viện này học được gì từ WangHongxi2001/PID_Library và SimpleFOC — và vì sao 2.0 bỏ bớt phần lớn những gì đã học.

---

## Giấy phép

MIT. Xem [LICENSE](LICENSE).

---
---

<a name="english"></a>

# shark_pid (English)

**The Simulink `PID Controller (2DOF)` block, reimplemented in portable C99 for microcontrollers.** Every parameter maps one-to-one onto a Block Parameters field, so the model you tune in simulation *is* the controller that runs on the hardware.

```
~1100 bytes Flash  ·  0 bytes static RAM  ·  C99  ·  no dynamic allocation  ·  no -lm  ·  MIT
```

## The problem it solves

You tune a PID in Simulink, the plot looks good, you flash it to an STM32 — and the system behaves differently. Usually not because the plant model was wrong, but because the *controller* in the simulation and the *controller* in the firmware are two different algorithms: different integrator discretisation, different derivative filter, and above all a different anti-windup circuit once the actuator saturates.

shark_pid closes that gap from the other direction: instead of adding features to a C library and then trying to simulate them, it transcribes the block diagram inside the PID block, and has nothing else.

| | |
|---|---|
| Parameters | 100% one-to-one with a Block Parameters field |
| Deviation from the block, unsaturated | `1e-15` *(double)* |
| Deviation from the block, **saturated** | `1e-15` *(double)* — where 1.x could not follow |
| Deviation you actually see on an MCU | `~4e-6` — the `float` rounding floor, irreducible |

[`extras/Test_Shark_PID/`](extras/Test_Shark_PID/) ships the cross-check: `verify_shark_vs_pid2` (Simulink S-Function running real C code against the native 2DOF PID block).

## Design principles

1. **Nothing the block does not have.** A feature you cannot simulate is a feature you cannot verify before flashing. Need a deadband, a rate limiter, feedforward? In Simulink those are `Dead Zone`, `Rate Limiter` and `Sum` blocks wired *after* the PID block — so here they live *outside* the controller too, in the calling layer.
2. **`dt` is an explicit argument.** `Ki` is in 1/second, `Kd` in seconds, `N` in rad/second. Move from 1 kHz to 50 Hz and the response barely changes. This is the one thing shark_pid has that the block does not, because the block's `Sample time` is a constant while a real timer interrupt jitters.
3. **No Arduino, no HAL.** The core needs only `<stdint.h>` and calls no `<math.h>` function, so it does not even need `-lm`. Builds for STM32, ESP-IDF, AVR, or runs on a PC.
4. **Never returns NaN, never divides by zero.** One bad ADC sample poisons a naive integrator forever; shark_pid rejects it at the door and holds the previous command.

## Quick start

```cpp
#include <SharkPID.hpp>
SharkPID pid(8.0f, 0.6f, 12.0f, 0.0f, 255.0f);   // Kp, Ki, Kd, min, max
pid.cfg().n = 3.33f;                              // Filter coefficient N
float u = pid.update(setpoint, measurement);      // dt measured via micros()
```

```c
#include "shark_pid.h"
shark_pid_cfg_t cfg;  shark_pid_cfg_default(&cfg);
cfg.kp = 4.0f;  cfg.ki = 1.5f;  cfg.kd = 0.1f;  cfg.n = 250.0f;
shark_pid_init(&pid, &cfg);
float u = shark_pid_update(&pid, setpoint, measurement, 0.001f);
```

## Why 2-DOF?

A traditional 1-DOF PID controller receives only the `error` ($e = r - y$), causing P and D terms to react violently to abrupt setpoint changes (overshoot and derivative kick). 

A **2-DOF** (Two Degrees of Freedom) architecture fixes this by receiving `setpoint` and `measurement` separately. This allows the use of **setpoint weights** (`b` and `c`) to tame the P and D terms during setpoint changes, without compromising the controller's ability to reject load disturbances. That is why the core library uses `shark_pid_update(&pid, setpoint, measurement, dt);`.


## Parameter mapping

| `shark_pid_cfg_t` | Block Parameters field |
|---|---|
| `kp`, `ki`, `kd` | `P`, `I`, `D` |
| `b`, `c` | `Setpoint weight (b)`, `Setpoint weight (c)` |
| `n` | `Filter coefficient (N)`; `n <= 0` unchecks `Use filtered derivative` |
| `TRAPEZOID_I` flag | `Integrator method` = `Trapezoidal` / `Backward Euler` |
| `out_min`, `out_max` | `Limit output` + saturation limits |
| `i_min`, `i_max` | `Limit integrator` + integrator saturation limits |
| `CLAMP_I` / `BACKCALC_I` + `kb` | `Anti-windup method` = `clamping` / `back-calculation` + `Kb` |
| `dt_max`, `dt_nominal` | *(no equivalent — the block's `Ts` is constant)* |

## Gotcha worth knowing: `b < 1`

At steady state the proportional term outputs $-K_p\,r\,(1-b)$, so **the integrator must supply exactly $K_p\,r\,(1-b)$** on top of the static load. With `kp=4`, `r=45`, `b=0.7` that alone is **54 units** — if `i_max` is below it you get a permanent steady-state error that looks nothing like a gain problem. Lower `b`, raise `i_max`.

## Verification

`test/test_shark_pid.c` is a dependency-free host test suite — no microcontroller, no test framework. Run it with `make -C test test`. Its first group embeds the block's standard difference equations (validated against the real Simulink R2022b block) and checks the C core against them across 15 configurations, so "matches the block" is guarded on every push.

CI on every push: the suite under both `gcc` and `clang` with `-Werror`, then again under AddressSanitizer + UBSan; cross-compilation for Cortex-M0 and Cortex-M4 with the full strict warning set and `-Werror`; Arduino Lint in `library-manager: submit` / `compliance: strict` mode; and sketch compilation for Arduino Uno.

## Upgrading from 1.x

`d_tau` became `n` (`n = 1/d_tau`), `kt` became `kb`, `i_term` is now the integrator's *output* while the state lives in `i_state`, and feedforward, deadband, changing-integral-rate, output filter, slew limiting and stall detection were removed — add them in the calling layer, exactly as you would wire an extra block after the PID block in Simulink. See [CHANGELOG.md](CHANGELOG.md).

## License

MIT — see [LICENSE](LICENSE).
