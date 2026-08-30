# shark_pid học được gì từ đâu

Thư viện này không sinh ra từ chỗ trống. Nó là kết quả đọc kỹ mã nguồn của hai
thư viện PID mã nguồn mở phổ biến, giữ lại những ý tưởng tốt và vá những chỗ hở.
Tài liệu này ghi lại chi tiết để bạn tự kiểm chứng chứ không phải tin lời.

> **Cập nhật cho 2.0.** shark_pid 2.0 đổi mục tiêu: nó là bản sao số của khối
> `PID Controller (2DOF)` trong Simulink, nên **phần lớn tính năng học được từ
> hai thư viện dưới đây đã bị gỡ bỏ** — vùng chết, biến tốc độ tích phân, lọc
> ngõ ra, giới hạn dốc, phát hiện kẹt, feedforward. Lý do và chỗ thay thế nằm
> ở [mục 4](#4-vì-sao-20-gỡ-bớt). Phần mổ xẻ lỗi thì giữ nguyên, vì những lỗi
> đó không cũ đi.

---

## 1. WangHongxi2001/PID_Library

Kho: <https://github.com/WangHongxi2001/PID_Library> — phiên bản `v1.0.6`,
commit cuối **12/05/2020**, chỉ có nhánh `master`.

Đây là code thi đấu RoboMaster, chạy trên STM32 với task chu kỳ cố định và động
cơ DJI. **Trong đúng ngữ cảnh đó nó hoạt động tốt.** Vấn đề chỉ xuất hiện khi
đem dùng như một thư viện đa dụng.

### Ý tưởng hay, shark_pid giữ lại

| Kỹ thuật | Ở bản gốc | Ở shark_pid 1.x | Ở shark_pid 2.0 |
|---|---|---|---|
| Tích phân hình thang | `f_Trapezoid_Intergral` | `SHARK_PID_F_TRAPEZOID_I` | giữ |
| Vi phân theo giá trị đo | `f_Derivative_On_Measurement` | `cfg.c = 0` | giữ |
| Lọc khâu D | `f_Derivative_Filter` | `cfg.d_tau` | `cfg.n` *(Filter coefficient N)* |
| Biến tốc độ tích phân A/B | `f_Changing_Integral_Rate` | `cfg.ci_a`, `cfg.ci_b` | **gỡ** |
| Lọc ngõ ra | `f_Output_Filter` | `cfg.out_tau` | **gỡ** |
| Phát hiện kẹt cơ cấu | `f_PID_ErrorHandle` | `cfg.stall_*` | **gỡ** |

Còn một chi tiết thiết kế tinh tế mà bản gốc làm đúng và ít ai để ý:

```c
pid->ITerm = pid->Ki * pid->Err;
pid->Iout += pid->ITerm;
```

Bộ tích luỹ **đã chứa sẵn `Ki`**. Nhờ vậy đổi `Ki` lúc đang chạy không làm ngõ ra
nhảy bậc, khác với kiểu `Iout = Ki * tổng_sai_số` vốn nhân lại toàn bộ lịch sử
bằng hệ số mới. shark_pid giữ nguyên cách này.

### Những chỗ shark_pid làm khác

#### a) Không hề có `dt`

```c
pid->ITerm = pid->Ki * pid->Err;                    /* thiếu * dt */
pid->Dout  = pid->Kd * (pid->Err - pid->Last_Err);  /* thiếu / dt */
```

Struct khai báo `ControlPeriod`, `thistime`, `lasttime`, `dtime` nhưng **không
dòng nào dùng tới**. Hệ quả: `Ki` và `Kd` dính chặt vào tần số vòng lặp. Đổi
1 kHz sang 500 Hz là `Ki` đổi 2 lần, `Kd` đổi một nửa. Nhịp bị jitter thì tích
phân sai. Hệ số lọc `alpha` cố định cũng vậy — tần số cắt trôi theo tốc độ loop.

Đo được: đổi tần số 20 lần thì thiết kế không có `dt` lệch đỉnh **2.51 °C**,
shark_pid lệch **0.002 °C**.

#### b) Macro `ABS` thiếu ngoặc

```c
#define ABS(x) ((x > 0) ? x : -x)
```

`ABS(pid->Target - pid->Measure)` nở ra thành:

```c
((pid->Target - pid->Measure > 0) ? pid->Target - pid->Measure
                                  : -pid->Target - pid->Measure)
```

Nhánh sai trả về $-T-M$ chứ không phải $|T-M|$. Sai hẳn khi `Measure > Target`.
Nó nằm ngay trong `f_PID_ErrorHandle`.

shark_pid không dùng macro cho việc này — chỉ hàm `static`. Lõi 2.0 thậm chí
không gọi hàm nào của `<math.h>`, nên không có chỗ nào để một macro như vậy len vào.

#### c) `f_PID_ErrorHandle` hỏng ở bốn điểm

```c
if (pid->Output < pid->MaxOut * 0.01f) return;
```
Động cơ chạy lùi (`Output` âm) luôn thoả điều kiện này nên thoát sớm — **không
bao giờ phát hiện được kẹt khi quay chiều âm.**

```c
if ((ABS(pid->Target - pid->Measure) / pid->Target) > 0.9f)
```
`Target = 0` là **chia cho 0**. Mà lệnh dừng động cơ chính là `Target = 0`.
Biến `LastNoneZeroTarget` được khai báo đúng để chữa lỗi này nhưng không bao giờ
được gán hay dùng.

```c
if (pid->ERRORHandler.ERRORCount > 1000)
```
Đếm 1000 **vòng lặp**, không có đơn vị thời gian. Ở 1 kHz là 1 giây, ở 100 Hz là
10 giây.

Và cờ lỗi **chốt vĩnh viễn** — không có API nào xoá, phải `PID_Init` lại.

shark_pid 1.x đếm bằng `stall_time` (giây), dùng `fabsf` nên bắt được cả hai
chiều, không có phép chia nào, và `shark_pid_clear_status()` xoá được. 2.0 gỡ
hẳn tính năng này ra khỏi bộ điều khiển — nó là logic của tầng ứng dụng, và
khối PID cũng không có. Cách làm lại: đếm thời gian `pid.isSaturated()` liên
tục đúng trong khi giá trị đo không nhúc nhích.

#### d) Cờ khai báo mà không cài đặt

`Proportional_On_Measurement = 0x08` có trong enum ở `pid.h`, nhưng trong
`pid.c` **không có nhánh `if` nào** xử lý nó. Bật cờ thì im lặng không có gì xảy ra.

shark_pid thay cả hai cờ *Derivative-on-Measurement* và *Proportional-on-Measurement*
bằng hai hệ số `b` và `c` — vừa tổng quát hơn, vừa không thể "khai báo mà quên cài".

#### e) `main.c` không biên dịch được

`PID_Init` có 12 tham số; ví dụ trong `main.c` truyền vào **13 đối số**. File
example đã lệch pha với header từ lâu.

#### f) Chống windup hở đúng lúc cần nhất

```c
if (ABS(temp_Output) > pid->MaxOut) {
    if (pid->Err * pid->Iout > 0) { pid->ITerm = 0; }
}
```

Điều kiện dựa trên dấu của **bộ tích luỹ**. Lúc khởi động `Iout = 0` nên tích
bằng 0, không thoả điều kiện, tích phân **vẫn cộng dồn dù ngõ ra đã bão hoà** —
tức là cơ chế mất tác dụng đúng vào lúc windup xảy ra mạnh nhất.

shark_pid xét dấu của **lượng bị cắt ở ngõ ra**, là trạng thái bão hoà thực.
Và có thêm back-calculation như lựa chọn mượt hơn.

2.0 đi thêm một bước: lượng vượt biên được đo trên đúng tín hiệu `preSat` mà
mạch anti-windup của khối PID nhìn (`P + TRẠNG THÁI khâu I + D`, chưa cộng nhịp
hiện tại), nên hai bên khớp cả ở những nhịp chạm biên. Xem
[`extras/Test_Shark_PID/`](../extras/Test_Shark_PID/).

#### g) Vụn vặt nhưng có thật

- Khai báo `static` trong file `.h` — mọi translation unit include vào đều bị
  cảnh báo "declared static but never defined".
- Macro `ABS` không có `#ifndef` guard, dễ đụng với HAL của ST.
- Con trỏ hàm `PID_param_init`/`PID_reset` nhét trong struct: tốn 8 byte mỗi
  instance và thêm một lần gọi gián tiếp, cho thứ không bao giờ đa hình.
- `MaxErr` tính rồi bỏ đó, không ai đọc.
- Không có hàm reset **trạng thái** — `PID_reset` chỉ đổi hệ số.
- Ngõ ra kẹp đối xứng `±MaxOut`, không có `out_min` riêng.

---

## 2. SimpleFOC (Arduino-FOC)

Kho: <https://github.com/simplefoc/Arduino-FOC> — file `src/common/pid.cpp`.

Gọn gàng, đúng đắn, và **có `dt` thật**. shark_pid học trực tiếp hai thứ:

- **Đo `dt` bằng `micros()` với phép trừ không dấu**, xử lý đúng cả khi bộ đếm
  32-bit tràn (~71,6 phút) → `SharkPID::update()` hai đối số
- **Giới hạn dốc ngõ ra** (`output_ramp`) → `cfg.out_slew` ở 1.x. 2.0 đưa ra
  ngoài lõi, xem `applySlew()` trong
  [`examples/02_Motor_Bidirectional`](../examples/02_Motor_Bidirectional)

> Ghi chú cho ai đọc bài 5 trên blog: phép trừ `timestamp_now - timestamp_prev`
> giữa hai số `unsigned long` **vẫn cho đúng khoảng cách khi tràn số**. Bộ bảo
> vệ `if (Ts <= 0 || Ts > 0.5f)` thực chất là để chặn nhịp đầu tiên và những lần
> lịch trình bị treo, chứ không phải để chữa tràn số.

Những chỗ shark_pid thêm vào:

| | SimpleFOC | shark_pid 2.0 |
|---|---|---|
| Lọc khâu D | không có | `cfg.n`, hệ số suy từ `dt` |
| Vi phân theo giá trị đo | không có — dùng sai số nên có derivative kick | `cfg.c = 0` |
| P theo giá trị đo | không có | `cfg.b` |
| Kẹp riêng khâu I | dùng chung `limit` với ngõ ra | `i_min` / `i_max` tách riêng |
| Back-calculation | không có | `SHARK_PID_F_BACKCALC_I`, `cfg.kb` |
| Chống NaN | không có | chặn ở cửa |
| Đối chiếu được với Simulink | không | 15/15 cấu hình, `1e-15` |
| Phụ thuộc Arduino | có (`Arduino.h`, `micros()`) | lõi C99 thuần |

Ngoài ra `integral = constrain(integral, -limit, limit)` của SimpleFOC dùng chung
`limit` cho cả tích phân lẫn ngõ ra. shark_pid tách `i_min/i_max` khỏi
`out_min/out_max` vì trần tích phân thường nên hẹp hơn trần ngõ ra.

---

## 3. Bảng tra nhanh: cờ cũ → tham số mới

Nếu bạn đang chuyển code từ `PID_Library` sang:

| Bản gốc | shark_pid 2.0 |
|---|---|
| `Integral_Limit` | `cfg.i_min` / `cfg.i_max` *(luôn có hiệu lực)* |
| `Derivative_On_Measurement` | `cfg.c = 0.0f` *(mặc định)* |
| `Trapezoid_Intergral` | `SHARK_PID_F_TRAPEZOID_I` |
| `Proportional_On_Measurement` | `cfg.b = 0.0f` *(bản gốc chưa cài đặt)* |
| `DerivativeFilter` | `cfg.n` *(Filter coefficient N, rad/giây — không phải alpha)* |
| `OutputFilter` | **không còn** — lọc thông thấp sau lệnh, ở tầng gọi |
| `ChangingIntegralRate` | **không còn** |
| `ErrorHandle` | **không còn** — đếm `pid.isSaturated()` ở tầng ứng dụng |

**Lưu ý khi chuyển hệ số:** bản gốc không nhân `dt` nên `Ki_cũ` tương ứng với
`Ki_mới × dt` và `Kd_cũ` tương ứng với `Kd_mới ÷ dt`. Nếu vòng lặp cũ chạy 1 kHz:

```
Ki_mới = Ki_cũ × 1000        Kd_mới = Kd_cũ ÷ 1000
```

Còn `alpha` lọc cũ đổi sang hệ số `N`: `N = alpha / ((1 - alpha) × dt)`.
Với `alpha = 0.25` ở 1 kHz thì `N = 333` (tương đương hằng số thời gian
`0.003` giây).

---

## 4. Vì sao 2.0 gỡ bớt

Ba tính năng học được ở trên — biến tốc độ tích phân, lọc ngõ ra, phát hiện kẹt
— cộng với vùng chết, giới hạn dốc và feedforward, đều **không có ô nào tương
ứng trong khối `PID Controller (2DOF)` của Simulink**.

Với một thư viện nhúng bình thường thì đó không phải vấn đề. Với một thư viện
mà lời hứa là *mô hình bạn dò trong mô phỏng chính là bộ điều khiển chạy trên
phần cứng* thì đó lại là vấn đề lớn nhất: bật `deadband` lên một cái là mô hình
Simulink lập tức không còn tả đúng firmware nữa, và bạn mất luôn khả năng kiểm
chứng trước khi nạp — đúng thứ mà bộ `extras/Test_Shark_PID/` sinh ra để làm.

Nên 2.0 chọn cách khác: giữ lõi đúng bằng khối, và đẩy phần còn lại ra tầng
gọi — đúng như trong Simulink chúng là những khối `Dead Zone`, `Rate Limiter`,
`Sum`, `Discrete Filter` nối *sau* khối PID. Bảng
[Những gì cố tình KHÔNG có](../README.md#những-gì-cố-tình-không-có) trong README
chỉ chỗ thay thế cho từng món, còn `examples/` và `extras/` cài sẵn hai món hay
dùng nhất — giới hạn dốc và feedforward — mỗi món chưa tới 10 dòng.
