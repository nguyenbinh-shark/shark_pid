# shark_pid

[![CI](https://github.com/nguyenbinh-shark/shark_pid/actions/workflows/ci.yml/badge.svg)](https://github.com/nguyenbinh-shark/shark_pid/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![C99](https://img.shields.io/badge/C-99-blue.svg)](src/shark_pid.c)

**Thư viện PID cho vi điều khiển, chạy giống hệt khối `PID Controller (2DOF)` của Simulink.**

Bạn dò tham số PID trong Simulink cho tới khi đồ thị đẹp. Bạn chép đúng những con số đó vào code. Phần cứng chạy ra đúng kết quả bạn vừa thấy trong mô phỏng. Đó là toàn bộ mục tiêu của thư viện này.

> *The Simulink PID Controller (2DOF) block, reimplemented in portable C99 for microcontrollers. [English summary below](#english).*

```
C99  ·  0 byte RAM tĩnh  ·  không cấp phát động  ·  không cần -lm  ·  MIT
```

Viết bởi [Trần Nguyên Bình](https://github.com/nguyenbinh-shark), như phần kết cho series [Thực Chiến Lập Trình PID](https://nguyenbinh-shark.github.io/pid-series/).

---

## Bạn nên đọc phần nào?

| Bạn là | Đọc từ |
|---|---|
| Mới học PID, chỉ muốn nó chạy | Mục 1 → 5, rồi mở [`examples/01_Heater_Basic`](examples/01_Heater_Basic) |
| Đã biết PID, cần biết thư viện có gì | Mục 6, 8, 9 |
| Đang dùng Simulink, muốn khớp mô phỏng với firmware | Mục 8 và 12 |
| Gặp lỗi, hệ chạy sai | Mục 10 |
| Gặp từ lạ trong tài liệu | Mục 14 — từ điển thuật ngữ |

### Mục lục

1. [shark_pid là gì?](#1-shark_pid-là-gì)
2. [Cài đặt](#2-cài-đặt)
3. [Chạy thử trong 5 phút](#3-chạy-thử-trong-5-phút)
4. [Ba con số quan trọng nhất: Kp, Ki, Kd](#4-ba-con-số-quan-trọng-nhất-kp-ki-kd)
5. [Chỉnh tham số theo thứ tự](#5-chỉnh-tham-số-theo-thứ-tự)
6. [Các tham số còn lại, giải thích bằng ngôn ngữ](#6-các-tham-số-còn-lại-giải-thích-bằng-ngôn-ngữ)
7. [Dùng với C thuần (STM32, ESP-IDF)](#7-dùng-với-c-thuần-stm32-esp-idf)
8. [Bảng ánh xạ sang khối Simulink](#8-bảng-ánh-xạ-sang-khối-simulink)
9. [Những gì thư viện cố tình không có](#9-những-gì-thư-viện-cố-tình-không-có)
10. [Hỏng thì xem ở đây](#10-hỏng-thì-xem-ở-đây)
11. [Chuyển tay sang tự động không giật](#11-chuyển-tay-sang-tự-động-không-giật)
12. [Tự kiểm chứng với Simulink](#12-tự-kiểm-chứng-với-simulink)
13. [Ví dụ đầy đủ](#13-ví-dụ-đầy-đủ)
14. [Từ điển thuật ngữ](#14-từ-điển-thuật-ngữ)
15. [Đọc thêm](#15-đọc-thêm)

---

## 1. shark_pid là gì?

### Nói ngắn

Một bộ điều khiển PID viết bằng C99. Bạn đưa vào **giá trị mong muốn** và **giá trị đo được**, nó trả về **lệnh điều khiển** — ví dụ mức PWM cho sò công suất hoặc động cơ.

```c
float u = shark_pid_update(&pid,
                           60.0f,    /* muốn 60 °C          */
                           42.5f,    /* đang đo được 42.5 °C */
                           0.05f);   /* 50 ms kể từ lần gọi trước */
```

### Điểm khác so với các thư viện PID khác

Hầu hết thư viện PID trên mạng đều tính ra một con số hợp lý. Vấn đề là **con số đó không giống con số Simulink tính ra** với cùng bộ tham số. Vì sao? Vì mỗi thư viện tự chọn một kiểu:

- cách cộng dồn khâu I (hình chữ nhật hay hình thang),
- cách lọc nhiễu cho khâu D,
- và quan trọng nhất: **xử lý thế nào khi cơ cấu chấp hành đã hết cỡ** — PWM đã lên 255 mà lò vẫn chưa đủ nóng.

shark_pid chép lại đúng sơ đồ bên trong khối `PID Controller (2DOF)` của Simulink, kể cả ba chỗ trên. Nên câu chuyện quen thuộc *"mô phỏng đẹp mà nạp xuống STM32 lại chạy khác"* không còn xảy ra vì bộ điều khiển nữa.

### Cái giá phải trả

Khối PID của Simulink không có tính năng gì thì thư viện này cũng không có tính năng đó. Không vùng chết, không giới hạn dốc, không feedforward. Không phải quên — là cố ý. Xem [mục 9](#9-những-gì-thư-viện-cố-tình-không-có) để biết cách tự thêm bằng vài dòng.

### Một khác biệt duy nhất so với khối Simulink

Trong Simulink, ô `Sample time` là một hằng số: cứ đúng 1 ms một lần, không sai một phần triệu giây.

Trên vi điều khiển thì không như vậy. Ngắt timer có thể bị trễ vì một ngắt khác đang chạy. Vì thế shark_pid bắt bạn **truyền vào `dt` — khoảng thời gian thật giữa hai lần gọi, tính bằng giây**. Đây là điểm duy nhất thư viện làm khác khối gốc, và làm khác vì phần cứng bắt buộc phải vậy.

---

## 2. Cài đặt

| Môi trường | Cách làm |
|---|---|
| **Arduino IDE** | Tải repo về dạng ZIP → `Sketch` → `Include Library` → `Add .ZIP Library...` → chọn file vừa tải |
| **PlatformIO** | Thêm vào `platformio.ini`: `lib_deps = https://github.com/nguyenbinh-shark/shark_pid.git` |
| **C thuần / STM32 / ESP-IDF** | Chép 2 file [`src/shark_pid.c`](src/shark_pid.c) và [`src/shark_pid.h`](src/shark_pid.h) vào dự án. Hết, không cần thêm gì |

Thư viện không cần link `-lm` (không dùng `<math.h>`), không dùng `malloc`, không có biến toàn cục.

---

## 3. Chạy thử trong 5 phút

Ví dụ dưới đây điều khiển nhiệt độ một lò sấy bằng Arduino: đọc cảm biến ở chân A0, xuất PWM ra chân 9.

```cpp
#include <SharkPID.hpp>

// Kp, Ki, Kd, giới hạn dưới, giới hạn trên.
// Sò công suất chỉ nung được chứ không làm lạnh, nên dải lệnh ra là 0..255.
SharkPID pid(8.0f, 0.6f, 12.0f, 0.0f, 255.0f);

void setup() {
  pinMode(9, OUTPUT);
}

void loop() {
  float nhietDo = analogRead(A0) * (5.0f / 1023.0f) * 100.0f;  // thay bằng cảm biến thật của bạn
  float u = pid.update(60.0f, nhietDo);   // muốn 60 °C; dt được tự đo bằng micros()
  analogWrite(9, (int)u);
}
```

### Đọc từng phần

| Dòng | Nghĩa là gì |
|---|---|
| `SharkPID pid(8.0f, 0.6f, 12.0f, 0.0f, 255.0f)` | Tạo bộ điều khiển với `Kp = 8`, `Ki = 0.6`, `Kd = 12`, lệnh ra bị kẹp trong khoảng `0…255` |
| `pid.update(60.0f, nhietDo)` | Tham số 1 là **giá trị mong muốn**, tham số 2 là **giá trị đo được**. Thứ tự này không được đảo |
| Bản `update` 2 tham số | Chỉ có trên Arduino. Nó tự đo `dt` bằng `micros()`. Tiện khi thử nghiệm |
| Bản `update` 3 tham số | `pid.update(sp, meas, dt)` — bạn tự đưa `dt` vào. **Nên dùng cái này** khi chạy trong task RTOS hay ngắt timer có chu kỳ cố định |

### Ba điều dễ sai nhất khi mới bắt đầu

1. **`dt` tính bằng giây, không phải mili giây.** Vòng lặp 50 ms thì `dt = 0.05f`, không phải `50.0f`.
2. **`out_min` / `out_max` phải khớp cơ cấu thật.** Sò nung một chiều thì `0…255`. Động cơ quay hai chiều qua cầu H thì `-255…255`.
3. **Đừng đặt `delay()` trong vòng lặp điều khiển.** Hãy so sánh `millis()` như trong [`examples/01_Heater_Basic`](examples/01_Heater_Basic).

---

## 4. Ba con số quan trọng nhất: Kp, Ki, Kd

Nếu bạn mới học PID, đây là phần cần nắm trước tất cả phần còn lại.

Gọi **sai số** là `e = giá trị mong muốn − giá trị đo được`.

| Khâu | Nhìn vào | Làm gì | Đặt quá lớn thì |
|---|---|---|---|
| **P** (`kp`) | Sai số **hiện tại** | Sai nhiều thì đẩy mạnh, sai ít thì đẩy nhẹ | Hệ dao động qua lại quanh đích |
| **I** (`ki`) | Sai số **tích luỹ theo thời gian** | Xoá phần sai số nhỏ dai dẳng mà P không dẹp nổi | Vọt lố cao, lâu về đích |
| **D** (`kd`) | **Tốc độ thay đổi** của giá trị đo | Phanh bớt lại khi đang lao nhanh tới đích | Ngõ ra rung và kêu, vì nhiễu bị khuếch đại |

### Đơn vị — chỗ này khác nhiều thư viện khác

- `ki` tính theo **1/giây**
- `kd` tính theo **giây**

Thư viện tự nhân và chia `dt` bên trong. Hệ quả rất tiện: **đổi tần số vòng lặp từ 1 kHz xuống 50 Hz thì đáp ứng gần như không đổi**, bạn không phải dò lại tham số. Nhiều thư viện khác không nhân `dt`, nên đổi tần số là phải dò lại từ đầu.

---

## 5. Chỉnh tham số theo thứ tự

Cách làm an toàn cho người mới. Làm từng bước, đừng chỉnh hai thứ cùng lúc.

**Bước 0 — chuẩn bị.** Đặt `out_min`, `out_max` đúng theo cơ cấu. In `setpoint`, giá trị đo và `u` ra Serial để nhìn được chuyện gì đang xảy ra.

**Bước 1 — chỉ có P.** Đặt `ki = 0`, `kd = 0`. Tăng dần `kp` cho tới khi hệ bắt đầu dao động nhẹ quanh đích, rồi **giảm xuống còn khoảng 60–70%** giá trị đó.

Lúc này hệ sẽ tới gần đích nhưng dừng lại trước đích một chút. Đó là chuyện bình thường — khâu I sẽ dẹp nó ở bước sau.

**Bước 2 — thêm I.** Tăng dần `ki` cho tới khi phần sai số còn lại bị xoá hết trong khoảng thời gian bạn chấp nhận được. Nếu hệ bắt đầu vọt quá đích: giảm `ki`, hoặc siết `i_min` / `i_max` lại (xem [mục 6.3](#63-i_min--i_max-và-chống-windup)).

**Bước 3 — thêm D nếu cần.** Chỉ thêm khi vẫn còn vọt lố mà bạn không muốn giảm tốc độ đáp ứng. Tăng `kd` từ giá trị nhỏ. Nếu ngõ ra bắt đầu rung hoặc động cơ kêu rít: giảm `kd`, hoặc **giảm `n`** để lọc nhiễu mạnh hơn.

**Bước 4 — dò trong Simulink cho nhanh (tuỳ chọn).** Nếu bạn có mô hình đối tượng trong Simulink, hãy dò ở đó rồi chép số sang theo [bảng ánh xạ ở mục 8](#8-bảng-ánh-xạ-sang-khối-simulink). Đó chính là lý do thư viện này tồn tại.

---

## 6. Các tham số còn lại, giải thích bằng ngôn ngữ

### 6.1. `b` và `c` — phần "2 bậc tự do" (2DOF)

**Vấn đề.** PID cổ điển chỉ nhìn thấy sai số `e`. Khi bạn đổi giá trị mong muốn đột ngột — ví dụ từ 30 °C nhảy lên 60 °C — sai số nhảy vọt 30 đơn vị chỉ trong một chu kỳ. Khâu P đẩy rất mạnh, gây vọt lố. Khâu D thì thấy "sai số đang đổi cực nhanh" nên bắn ra một xung khổng lồ. Xung đó gọi là **derivative kick**, và nó vô nghĩa: hệ thống có chạy nhanh đâu, chỉ có cái đích vừa bị dời đi thôi.

**Cách sửa.** PID hai bậc tự do nhận `setpoint` và `measurement` **tách riêng**, thay vì chỉ nhận hiệu của chúng. Nhờ vậy bạn có thêm hai núm vặn:

```
u = Kp·(b·r − y)  +  Ki·∫(r − y)dt  +  KhâuD(c·r − y)
```

với `r` là giá trị mong muốn, `y` là giá trị đo được.

| Núm | Đặt | Kết quả |
|---|---|---|
| `c` | `0` *(mặc định)* | Khâu D chỉ nhìn cảm biến, hoàn toàn không thấy setpoint → **hết derivative kick**. Gần như luôn muốn cái này |
| `c` | `1` | Khâu D nhìn sai số, kiểu PID cổ điển |
| `b` | `1` *(mặc định)* | Khâu P bám setpoint đầy đủ, đáp ứng nhanh nhất |
| `b` | `< 1` | Đổi setpoint êm hơn, ít vọt lố hơn, đổi lại chậm hơn |

Điểm hay: hai núm này chỉ ảnh hưởng lúc **đổi setpoint**. Khả năng chống nhiễu — ví dụ có người mở cửa lò — **không bị yếu đi**. Đó là điều PID một bậc tự do không làm được: muốn êm khi đổi setpoint thì phải giảm `Kp`, mà giảm `Kp` thì chống nhiễu kém theo.

> ⚠️ **Bẫy khi đặt `b < 1`** — đọc kỹ chỗ này, nó đã làm nhiều người mất cả buổi.
>
> Khi hệ đã ổn định (`y = r`), khâu P không xuất ra 0 nữa mà xuất ra `−Kp·r·(1−b)`, tức một số âm. Nghĩa là **khâu I phải bù đúng `Kp·r·(1−b)`**, cộng thêm phần bù tải bình thường.
>
> Ví dụ `kp = 4`, `r = 45`, `b = 0.7`: riêng phần này đã là `4 × 45 × 0.3 = 54` đơn vị. Nếu `i_max` của bạn nhỏ hơn 54, khâu I không bao giờ bù đủ, và hệ **sai số vĩnh viễn, không bao giờ tới đích** — trong khi nhìn ba hệ số PID thì thấy hoàn toàn bình thường.
>
> **Quy tắc: hạ `b` thì phải nới `i_max`.**

### 6.2. `n` — bộ lọc cho khâu D

Khâu D khuếch đại nhiễu cảm biến rất mạnh, nên hầu như luôn phải lọc.

`n` chính là ô `Filter coefficient (N)` của khối Simulink, đơn vị rad/giây, mặc định `100`.

| Muốn gì | Đặt `n` | Ghi nhớ |
|---|---|---|
| Lọc mạnh (cảm biến nhiễu, tín hiệu chậm như nhiệt độ) | Nhỏ, ví dụ `3.33` | `n` nhỏ = lọc mạnh = khâu D chậm |
| Lọc nhẹ (encoder sạch, tín hiệu nhanh) | Lớn, ví dụ `250` | `n` lớn = lọc nhẹ = khâu D nhạy |
| Không lọc | `<= 0` | Chỉ nên dùng khi tín hiệu cực sạch |

**Mẹo quy đổi:** hằng số thời gian của bộ lọc là `1/n` giây.
`n = 100` → 10 ms. `n = 3.33` → 300 ms. `n = 250` → 4 ms.

Hệ số lọc được tính lại từ `dt` ở **mỗi chu kỳ**, nên đổi tần số vòng lặp không làm trôi tần số cắt. Cách viết cứng kiểu `alpha = 0.25` trong nhiều thư viện khác thì bị trôi.

### 6.3. `i_min` / `i_max` và chống windup

**Windup là gì?** Hãy tưởng tượng lò sấy đang nguội 20 °C, bạn đặt đích 60 °C. PWM lên 255 ngay lập tức rồi đứng đó suốt 5 phút. Trong 5 phút đó, khâu I vẫn cần mẫn cộng dồn sai số, tích luỹ thành một con số khổng lồ — dù việc đó chẳng giúp ích gì, vì PWM đã hết cỡ rồi.

Tới khi nhiệt độ chạm 60 °C, khâu I vẫn đang giữ con số khổng lồ đó, nên PWM vẫn giữ 255. Lò vọt lên 80 °C rồi mới chịu quay xuống. Đó là **windup**.

Thư viện chống windup bằng hai lớp:

**Lớp 1 — kẹp cứng khâu I.** Đặt `i_min` / `i_max`. Khâu I không bao giờ được vượt ra ngoài khoảng này.

**Lớp 2 — cơ chế chống windup.** Chọn một trong hai bằng cờ:

| Cờ | Cơ chế | Khi nào dùng |
|---|---|---|
| `SHARK_PID_F_CLAMP_I` | Ngưng cộng dồn khâu I khi lệnh ra đã chạm biên và việc cộng thêm chỉ làm nó vượt xa hơn | **Mặc định. Người mới cứ dùng cái này** |
| `SHARK_PID_F_BACKCALC_I` | Kéo khâu I ngược trở lại theo lượng bị cắt, tốc độ kéo do `kb` quyết định | Khi muốn mượt hơn, không có kiểu bật/tắt đột ngột |

Khối Simulink chỉ chọn được **một** phương pháp. Nếu bạn bật cả hai cờ thì clamping thắng.

Với back-calculation: `kb` càng lớn thì kéo về càng nhanh. Khởi điểm thử `kb ≈ ki/kp` rồi tăng dần.

### 6.4. `dt_max` và `dt_nominal` — lưới an toàn

Hai tham số này **không có trong khối Simulink**, vì `Sample time` của khối là một hằng số hoàn hảo. Chúng chỉ tồn tại để bảo vệ firmware thật.

| Tình huống | Thư viện làm gì |
|---|---|
| `dt` âm, bằng 0, là NaN, hoặc lớn hơn `dt_max` | Thay bằng `dt_nominal` và bật cờ `SHARK_PID_BAD_DT`. Kiểm tra bằng `pid.hadBadDt()` |
| `setpoint` hoặc `measurement` là NaN / vô cực — cảm biến đứt dây chẳng hạn | **Bỏ qua nguyên chu kỳ đó**, giữ nguyên lệnh cũ, bật cờ `SHARK_PID_BAD_INPUT`. Khâu I không bị đầu độc |

Mặc định: `dt_max = 0.5` giây, `dt_nominal = 0.001` giây. Nên đặt `dt_nominal` bằng đúng chu kỳ vòng lặp thật của bạn.

### 6.5. Bảng tra nhanh toàn bộ tham số

| Trường | Mặc định | Ý nghĩa |
|---|---|---|
| `kp`, `ki`, `kd` | `1`, `0`, `0` | Ba hệ số PID. `ki` theo 1/giây, `kd` theo giây |
| `b` | `1.0` | Trọng số setpoint cho khâu P |
| `c` | `0.0` | Trọng số setpoint cho khâu D. `0` = chặn derivative kick |
| `n` | `100.0` | Hệ số lọc khâu D (rad/giây). `<= 0` = không lọc |
| `out_min`, `out_max` | `-100`, `100` | Dải lệnh ra, đặt theo cơ cấu thật |
| `i_min`, `i_max` | `-100`, `100` | Trần riêng cho khâu I |
| `kb` | `0.0` | Hệ số back-calculation, chỉ dùng khi bật cờ tương ứng |
| `dt_max` | `0.5` | Ngưỡng coi `dt` là bất thường (giây) |
| `dt_nominal` | `0.001` | `dt` thay thế khi bất thường (giây) |
| `flags` | `TRAPEZOID_I` + `CLAMP_I` | Các lựa chọn hành vi |

Toàn bộ API — 8 hàm, mỗi trường một dòng chú thích — nằm trong [`src/shark_pid.h`](src/shark_pid.h).

---

## 7. Dùng với C thuần (STM32, ESP-IDF)

Lõi thư viện không phụ thuộc Arduino hay HAL nào cả. Chép 2 file vào là dùng được.

```c
#include "shark_pid.h"

static shark_pid_t pid;

void control_init(void)
{
    shark_pid_cfg_t cfg;
    shark_pid_cfg_default(&cfg);        /* luôn gọi hàm này trước, để mọi trường có giá trị an toàn */

    cfg.kp = 4.0f;
    cfg.ki = 1.5f;                      /* 1/giây */
    cfg.kd = 0.1f;                      /* giây   */
    cfg.n  = 250.0f;                    /* lọc D nhẹ, hằng số thời gian 4 ms */

    cfg.out_min = -100.0f;              /* cơ cấu hai chiều */
    cfg.out_max =  100.0f;
    cfg.i_min   =  -60.0f;              /* trần khâu I hẹp hơn trần lệnh ra */
    cfg.i_max   =   60.0f;

    cfg.dt_nominal = 0.001f;            /* vòng lặp 1 kHz */

    shark_pid_init(&pid, &cfg);
}

/* Gọi trong ngắt timer 1 kHz */
float control_step(float target, float measured)
{
    return shark_pid_update(&pid, target, measured, 0.001f);
}
```

Nếu timer của bạn có jitter đáng kể, hãy đo `dt` thật — bằng DWT cycle counter hoặc một timer chạy tự do — rồi truyền vào thay cho hằng số `0.001f`.

---

## 8. Bảng ánh xạ sang khối Simulink

Đây là bảng bạn cần khi chép tham số qua lại giữa Simulink và code.

Cấu hình khối: `PID Controller (2DOF)`, `Form = Parallel`, `Time domain = Discrete-time`.

| Trong code (`shark_pid_cfg_t`) | Ô trong Block Parameters của Simulink |
|---|---|
| `dt` *(đối số của `update`)* | `Sample time` |
| `kp`, `ki`, `kd` | `P`, `I`, `D` |
| `b`, `c` | `Setpoint weight (b)`, `Setpoint weight (c)` |
| `n > 0` | Tích `Use filtered derivative`, điền `Filter coefficient (N)` = `n`, chọn `Filter method = Backward Euler` |
| `n <= 0` | Bỏ tích `Use filtered derivative` |
| Cờ `TRAPEZOID_I` bật / tắt | `Integrator method` = `Trapezoidal` / `Backward Euler` |
| `out_min`, `out_max` | Tích `Limit output`, điền `Lower/Upper saturation limit` |
| `i_min`, `i_max` | Tích `Limit integrator`, điền `Lower/Upper integrator saturation limit` |
| Cờ `CLAMP_I` | `Anti-windup method` = `clamping` |
| Cờ `BACKCALC_I` + `kb` | `Anti-windup method` = `back-calculation`, `Kb` = `kb` |
| `dt_max`, `dt_nominal` | *Không có ô tương ứng* — đây là lưới an toàn chỉ firmware mới cần |

---

## 9. Những gì thư viện cố tình không có

Nguyên tắc: **khối PID của Simulink không có thì thư viện cũng không có.**

Lý do rất thực tế: một tính năng không mô phỏng được là một tính năng không kiểm chứng được trước khi nạp firmware. Chỉ cần bật một tính năng "ngoài luồng", mô hình Simulink không còn tả đúng code nữa, và bạn mất luôn thứ quý nhất mà thư viện này mang lại.

Những thứ đó không biến mất — chúng chuyển ra tầng gọi, đúng như trong Simulink bạn nối thêm một khối **phía sau** khối PID:

| Bạn cần | Trong Simulink là khối | Trong code viết thế nào |
|---|---|---|
| Feedforward (bù trước) | `Sum` | `u = shark_pid_update(...) + ff;` *(nhớ chừa khoảng trống cho chống windup)* |
| Giới hạn dốc `du/dt` | `Rate Limiter` | Hàm `applySlew()` trong [ví dụ 02](examples/02_Motor_Bidirectional) — đúng 8 dòng |
| Vùng chết (dead zone) | `Dead Zone` | Một câu `if` quanh sai số |
| Lọc lệnh ra | `Discrete Filter` | Một bộ lọc thông thấp đặt sau lệnh |
| Phát hiện cơ cấu bị kẹt | Logic riêng | Đếm thời gian `pid.isSaturated()` liên tục ở tầng ứng dụng |

---

## 10. Hỏng thì xem ở đây

| Triệu chứng | Nguyên nhân thường gặp | Cách sửa |
|---|---|---|
| Hệ dừng lại trước đích, không bao giờ tới nơi | `ki = 0`, hoặc `i_max` quá nhỏ | Tăng `ki`; nới `i_max` |
| ...và bạn có đặt `b < 1` | Đúng cái bẫy ở [mục 6.1](#61-b-và-c--phần-2-bậc-tự-do-2dof) | Nới `i_max` lên trên `Kp·r·(1−b)`, hoặc trả `b = 1` |
| Vọt lố rất cao khi khởi động từ xa | Windup | Đặt `i_min` / `i_max` hợp lý, giữ cờ `CLAMP_I` |
| Ngõ ra rung, động cơ kêu rít | Khâu D khuếch đại nhiễu | Giảm `n` (lọc mạnh hơn), hoặc giảm `kd` |
| Lệnh nhảy vọt đúng khoảnh khắc đổi setpoint | Derivative kick | Đảm bảo `c = 0` (mặc định). Cân nhắc `b < 1` |
| Hệ chạy xa dần khỏi đích, càng lúc càng tệ | Dấu bị ngược | Đảo dây động cơ, hoặc đảo dấu giá trị đo |
| `u` luôn dính ở `out_max` | Cơ cấu quá yếu, hoặc sai đơn vị | Kiểm tra cơ cấu, kiểm tra thang đo cảm biến |
| Đổi tần số vòng lặp thì đáp ứng đổi hẳn | `dt` truyền vào sai đơn vị | `dt` tính bằng **giây**: 50 ms là `0.05f` |
| `pid.hadBadDt()` liên tục bật | `dt` vượt `dt_max`, hoặc vòng lặp bị treo | Kiểm tra chu kỳ thật; chỉnh `dt_max` cho đúng |
| `pid.hadBadInput()` bật | Cảm biến trả NaN / vô cực | Kiểm tra dây và hàm đọc cảm biến |
| Đổi hệ số lúc đang chạy làm lệnh ra nhảy bậc | Không xảy ra với thư viện này | Xem [mục 11](#11-chuyển-tay-sang-tự-động-không-giật) để biết vì sao |

---

## 11. Chuyển tay sang tự động không giật

Tình huống: bạn đang chỉnh PWM bằng tay ở mức 120, giờ muốn bật chế độ tự động. Nếu bật thẳng, khâu I đang bằng 0 nên lệnh rớt về gần 0 — cơ cấu giật một cái rất khó chịu.

Cách xử lý: nạp trước cho khâu I giá trị hiện tại.

```c
shark_pid_preload(&pid, 120.0f);   /* lệnh tay đang là 120 */
```

Lệnh tự động đầu tiên sẽ bám sát 120 rồi mới điều chỉnh dần. Việc này tương đương điền ô `Integrator Initial condition` trong tab `Initialization` của khối Simulink.

**Một điểm cộng miễn phí:** biến `i_state` bên trong lưu sẵn `Ki·∫e`, tức đã nhân `Ki` rồi. Nhờ vậy **đổi hệ số lúc đang chạy không làm lệnh ra nhảy bậc** — hệ số mới chỉ ảnh hưởng các bước tính về sau. Cách cài đặt ngây thơ kiểu `i = Ki × tổng(e)` thì nhân lại toàn bộ lịch sử bằng hệ số mới, nên vừa đổi `Ki` là lệnh ra nhảy dựng.

---

## 12. Tự kiểm chứng với Simulink

Bạn không cần tin lời tôi rằng thư viện khớp với khối Simulink. Thư mục [`extras/Test_Shark_PID/`](extras/Test_Shark_PID/) có sẵn bộ công cụ để bạn tự kiểm tra.

Cách nó hoạt động: đúng file `src/shark_pid.c` sẽ nạp lên vi điều khiển được biên dịch thành một khối chạy được trong Simulink, đặt song song với khối `PID Controller (2DOF)` thật, cho ăn cùng một tín hiệu vào, rồi so hai lệnh ra.

Trong cửa sổ lệnh MATLAB:

```matlab
cd extras/Test_Shark_PID
verify_shark_vs_pid2('kp', 4, 'ki', 1.5, 'kd', 0.1, 'n', 250, 'Ts', 0.001);
```

Script tự biên dịch, tự dựng mô hình, in ra sai số lớn nhất và vẽ đồ thị. Ngưỡng đạt là sai số tương đối `1e-5` — đây là sàn làm tròn khi so kiểu `float` của C với kiểu `double` của Simulink, không thể nhỏ hơn được.

Hướng dẫn chi tiết kèm video: [README của thư mục đó](extras/Test_Shark_PID/README.md).

### Không có MATLAB? Kiểm chứng ngay trên máy tính

Thư mục [`test/`](test/) có bộ test viết bằng C thuần: không cần vi điều khiển, không cần MATLAB, không phụ thuộc thư viện test nào.

```bash
cd test
make test          # biên dịch và chạy
make asan          # chạy lại dưới AddressSanitizer + UBSan
```

**Nhóm test số 0** làm đúng việc mà `verify_shark_vs_pid2.m` làm trong Simulink, nhưng bằng C: nó dựng lại phương trình sai phân của khối `PID Controller (2DOF)` bằng kiểu `double`, độc lập với lõi thư viện, rồi so từng nhịp trên 17 cấu hình — gồm cả bão hoà, kẹp khâu I, hai kiểu tích phân và hai kiểu chống windup. Sai số lớn nhất đo được là `3.3e-6`, đúng sàn làm tròn của `float`.

Mười nhóm còn lại kiểm tra hành vi: độc lập tần số lấy mẫu, chống windup, kẹp khâu I, derivative kick, bộ lọc `N`, chặn NaN/Inf, đổi hệ số không giật, preload, bất biến API, và bẫy `b < 1` ở [mục 6.1](#61-b-và-c--phần-2-bậc-tự-do-2dof).

Tổng cộng 66 phép kiểm tra. CI chạy toàn bộ với `gcc` và `clang` ở `-Werror`, rồi chạy lại dưới AddressSanitizer và UBSan.

---

## 13. Ví dụ đầy đủ

| Thư mục | Bài toán | Học được gì |
|---|---|---|
| [`examples/01_Heater_Basic`](examples/01_Heater_Basic) | Lò sấy, cơ cấu một chiều | Lọc D bằng `n` nhỏ, kẹp khâu I, chống windup kiểu clamping, in trạng thái ra Serial |
| [`examples/02_Motor_Bidirectional`](examples/02_Motor_Bidirectional) | Động cơ DC hai chiều qua cầu H | Lệnh ra âm, back-calculation, và cách tự thêm giới hạn dốc ở tầng gọi |
| [`extras/Test_Shark_PID`](extras/Test_Shark_PID) | Đối chiếu với Simulink | Cách chứng minh firmware khớp mô phỏng |
| [`test`](test) | Bộ test chạy trên PC | 66 phép kiểm tra, gồm nhóm đối chiếu với phương trình của khối |

Người mới nên bắt đầu từ ví dụ 01.

---

## 14. Từ điển thuật ngữ

| Từ | Nghĩa |
|---|---|
| **Setpoint** | Giá trị bạn muốn hệ đạt tới. Ví dụ 60 °C |
| **Measurement** | Giá trị cảm biến đang đo được |
| **Sai số (error)** | `setpoint − measurement` |
| **Lệnh điều khiển (`u`)** | Con số PID trả về, đưa ra cơ cấu chấp hành. Ví dụ mức PWM |
| **Cơ cấu chấp hành (actuator)** | Thứ tác động vào hệ: sò công suất, động cơ, van |
| **Bão hoà (saturation)** | Lệnh đã chạm giới hạn `out_min` hoặc `out_max`, không đẩy mạnh hơn được nữa |
| **Windup** | Khâu I cộng dồn vô ích trong lúc đang bão hoà, gây vọt lố nặng về sau. Xem [mục 6.3](#63-i_min--i_max-và-chống-windup) |
| **Derivative kick** | Xung nhọn ở khâu D mỗi khi setpoint bị đổi đột ngột. Xem [mục 6.1](#61-b-và-c--phần-2-bậc-tự-do-2dof) |
| **Vọt lố (overshoot)** | Hệ vượt quá đích rồi mới quay lại |
| **Sai số xác lập** | Phần sai số nhỏ còn lại vĩnh viễn khi hệ đã ổn định |
| **2-DOF** | Hai bậc tự do. Bộ điều khiển nhận setpoint và measurement tách riêng, nên chỉnh được đáp ứng theo setpoint mà không đụng tới khả năng chống nhiễu |
| **`dt`** | Khoảng thời gian giữa hai lần gọi hàm `update`, tính bằng **giây** |
| **Jitter** | Chu kỳ vòng lặp không đều, lúc 1.0 ms lúc 1.3 ms |
| **Bumpless transfer** | Chuyển từ điều khiển tay sang tự động mà cơ cấu không giật. Xem [mục 11](#11-chuyển-tay-sang-tự-động-không-giật) |
| **S-Function** | Cơ chế của Simulink cho phép gọi thẳng mã C từ trong mô hình. Dùng ở [mục 12](#12-tự-kiểm-chứng-với-simulink) |

---

## 15. Đọc thêm

Series **Thực Chiến Lập Trình PID** giải thích cặn kẽ từng kỹ thuật trong thư viện này:

1. [Từ công thức đến code](https://nguyenbinh-shark.github.io/posts/2026/08/pid-code-1-tu-cong-thuc-den-code/)
2. [5 căn bệnh của PID đầu tay](https://nguyenbinh-shark.github.io/posts/2026/08/pid-code-2-cac-benh-pid-dau-tay/)
3. [Tinh chỉnh I và D](https://nguyenbinh-shark.github.io/posts/2026/08/pid-code-3-tinh-chinh-i-va-d/)
4. [Đóng gói thư viện](https://nguyenbinh-shark.github.io/posts/2026/08/pid-code-4-dong-goi-thu-vien/)
5. [Đọc hiểu PID của người đi làm](https://nguyenbinh-shark.github.io/posts/2026/08/pid-code-5-doc-pid-nguoi-di-lam/)
6. [Cấu hình khối PID Simulink "chuẩn vị" code C](https://nguyenbinh-shark.github.io/posts/2026/08/simscape-multibody-pid-simulink-sim2real/) — bài mà thư viện này là kết luận

[`docs/so-sanh.md`](docs/so-sanh.md) kể lại thư viện học được gì từ hai thư viện PID mã nguồn mở phổ biến, và vì sao nó cố tình không mang theo phần lớn những tính năng đó.

---

## Giấy phép

MIT. Xem [LICENSE](LICENSE).

---
---

<a name="english"></a>

# shark_pid (English)

**A PID library for microcontrollers that behaves exactly like the Simulink `PID Controller (2DOF)` block.**

You tune the PID in Simulink until the plot looks right. You copy those numbers into your code. The hardware behaves the way the simulation just showed you.

```
C99  ·  0 bytes static RAM  ·  no dynamic allocation  ·  no -lm  ·  MIT
```

## Why this exists

Most PID libraries compute a reasonable number — but not *the same* number Simulink computes from the same gains. They differ in how the integrator is discretised, how the derivative is filtered, and above all in what happens once the actuator saturates.

shark_pid transcribes the block diagram inside the Simulink PID block, saturation circuit included. So the familiar story — *"the simulation looked great, then the STM32 behaved differently"* — stops being about the controller.

The price: anything the block does not have, this library does not have either. Dead zone, rate limiting and feedforward live in the calling layer, exactly as you would wire an extra block *after* the PID block in Simulink.

**The one deliberate difference:** `dt` is an explicit argument, because the block's `Sample time` is a perfect constant while a real timer interrupt jitters. `Ki` is in 1/second and `Kd` in seconds, so moving from 1 kHz to 50 Hz barely changes the response.

## Install

| Environment | How |
|---|---|
| Arduino IDE | `Sketch` → `Include Library` → `Add .ZIP Library...`, point it at this repo |
| PlatformIO | `lib_deps = https://github.com/nguyenbinh-shark/shark_pid.git` |
| Plain C / STM32 / ESP-IDF | Copy `src/shark_pid.c` and `src/shark_pid.h` into your project |

## Quick start

```cpp
#include <SharkPID.hpp>

SharkPID pid(8.0f, 0.6f, 12.0f, 0.0f, 255.0f);   // Kp, Ki, Kd, out_min, out_max

void loop() {
  float u = pid.update(60.0f, readSensor());     // dt measured via micros()
  analogWrite(9, (int)u);
}
```

```c
#include "shark_pid.h"

shark_pid_cfg_t cfg;
shark_pid_cfg_default(&cfg);                     // always start from the safe defaults
cfg.kp = 4.0f;  cfg.ki = 1.5f;  cfg.kd = 0.1f;  cfg.n = 250.0f;
shark_pid_init(&pid, &cfg);

float u = shark_pid_update(&pid, setpoint, measurement, 0.001f);   // dt in SECONDS
```

## What each parameter does

| Field | Meaning |
|---|---|
| `kp`, `ki`, `kd` | The three gains. `ki` in 1/second, `kd` in seconds — `dt` is applied internally |
| `b` | Setpoint weight for the P term. `1` = fastest response, `<1` = gentler setpoint changes |
| `c` | Setpoint weight for the D term. `0` (default) kills derivative kick |
| `n` | Derivative filter coefficient, rad/s. Smaller = heavier filtering. Time constant is `1/n` seconds. `<= 0` disables the filter |
| `out_min`, `out_max` | Command limits — match your actuator |
| `i_min`, `i_max` | Separate ceiling for the integrator |
| `kb` | Back-calculation coefficient, only used with the `BACKCALC_I` flag |
| `dt_max`, `dt_nominal` | Safety net for a jittering or stalled control loop |

## Why 2-DOF?

A 1-DOF PID only ever sees the error `e = r − y`, so an abrupt setpoint change makes P push hard (overshoot) and makes D fire a meaningless spike (derivative kick). A 2-DOF controller takes `setpoint` and `measurement` separately, so the setpoint weights `b` and `c` can tame both **without weakening disturbance rejection**. Hence the API: `shark_pid_update(&pid, setpoint, measurement, dt)`.

## Parameter mapping

Block configuration: `PID Controller (2DOF)`, `Form = Parallel`, `Time domain = Discrete-time`.

| `shark_pid_cfg_t` | Block Parameters field |
|---|---|
| `dt` *(argument)* | `Sample time` |
| `kp`, `ki`, `kd` | `P`, `I`, `D` |
| `b`, `c` | `Setpoint weight (b)`, `Setpoint weight (c)` |
| `n` | `Filter coefficient (N)`; `n <= 0` unchecks `Use filtered derivative` |
| `TRAPEZOID_I` flag | `Integrator method` = `Trapezoidal` / `Backward Euler` |
| `out_min`, `out_max` | `Limit output` + saturation limits |
| `i_min`, `i_max` | `Limit integrator` + integrator saturation limits |
| `CLAMP_I` / `BACKCALC_I` + `kb` | `Anti-windup method` = `clamping` / `back-calculation` + `Kb` |
| `dt_max`, `dt_nominal` | *(no equivalent — the block's `Ts` is a constant)* |

## The one gotcha you must know: `b < 1`

At steady state (`y = r`) the proportional term outputs `−Kp·r·(1−b)`, so **the integrator has to supply exactly `Kp·r·(1−b)`** on top of the static load.

With `kp = 4`, `r = 45`, `b = 0.7` that alone is 54 units. If `i_max` is below 54, you get a permanent steady-state error that looks nothing like a gain problem. **Lower `b`, raise `i_max`.**

## Verification

[`extras/Test_Shark_PID/`](extras/Test_Shark_PID/) compiles the very same `src/shark_pid.c` that ships to your microcontroller into a C MEX S-Function, runs it inside Simulink side by side with the native `PID Controller (2DOF)` block on the same input, and reports the difference:

```matlab
cd extras/Test_Shark_PID
verify_shark_vs_pid2('kp', 4, 'ki', 1.5, 'kd', 0.1, 'n', 250, 'Ts', 0.001);
```

The pass threshold is `1e-5` relative — the rounding floor of C `float` against Simulink `double`.

## License

MIT — see [LICENSE](LICENSE).
