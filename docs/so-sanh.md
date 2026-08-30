# shark_pid học được gì, từ đâu

Thư viện này không tự nhiên mà có. Nó là kết quả của việc đọc kỹ mã nguồn hai
thư viện PID mã nguồn mở phổ biến, giữ lại ý tưởng hay và tránh những chỗ hở.

Trang này ghi lại chuyện đó, kèm đường dẫn tới từng dòng code cụ thể, để bạn tự
kiểm chứng chứ không phải tin lời tôi.

**Bạn nên đọc trang này nếu:**

- bạn đang dùng một trong hai thư viện đó và muốn chuyển sang shark_pid,
- hoặc bạn tò mò vì sao shark_pid lại chọn cách làm như hiện tại.

Nếu bạn chỉ muốn dùng thư viện, [README](../README.md) là đủ.

---

## Trước hết: phạm vi của thư viện này

shark_pid không cố trở thành "bộ PID nhúng đầy đủ tính năng". Nó là **bản sao
số của khối `PID Controller (2DOF)` trong Simulink**, không hơn.

Hệ quả: **nhiều kỹ thuật hay của hai thư viện dưới đây cố tình không có mặt
trong lõi** — vùng chết, biến tốc độ tích phân, lọc lệnh ra, giới hạn dốc, phát
hiện kẹt cơ cấu, feedforward.

Chúng không bị bỏ quên; chúng thuộc về tầng gọi. Lý do đầy đủ ở
[mục 4](#4-vì-sao-thư-viện-không-có-những-tính-năng-đó).

### Mục lục

1. [WangHongxi2001/PID_Library](#1-wanghongxi2001pid_library)
2. [SimpleFOC (Arduino-FOC)](#2-simplefoc-arduino-foc)
3. [Bảng tra: chuyển code từ PID_Library sang shark_pid](#3-bảng-tra-chuyển-code-từ-pid_library-sang-shark_pid)
4. [Vì sao thư viện không có những tính năng đó](#4-vì-sao-thư-viện-không-có-những-tính-năng-đó)

---

## 1. WangHongxi2001/PID_Library

Kho: <https://github.com/WangHongxi2001/PID_Library> — phiên bản `v1.0.6`,
commit cuối ngày 12/05/2020.

**Đây là code gì?** Code thi đấu RoboMaster: chạy trên STM32, trong một task có
chu kỳ cố định, điều khiển động cơ DJI. **Trong đúng ngữ cảnh đó nó hoạt động
tốt.** Vấn đề chỉ xuất hiện khi ai đó đem nó dùng như một thư viện đa dụng cho
bài toán khác.

### 1.1. Những gì shark_pid học được

| Kỹ thuật | Tên ở bản gốc | Tương đương ở shark_pid |
|---|---|---|
| Tích phân hình thang | `f_Trapezoid_Intergral` | Cờ `SHARK_PID_F_TRAPEZOID_I` |
| Khâu D nhìn giá trị đo | `f_Derivative_On_Measurement` | `cfg.c = 0` |
| Lọc khâu D | `f_Derivative_Filter` | `cfg.n` *(ô Filter coefficient N)* |
| Biến tốc độ tích phân A/B | `f_Changing_Integral_Rate` | **không có** |
| Lọc lệnh ra | `f_Output_Filter` | **không có** |
| Phát hiện kẹt cơ cấu | `f_PID_ErrorHandle` | **không có** |

Có một chi tiết bản gốc làm rất đúng mà ít người để ý: **bộ tích luỹ của nó đã
chứa sẵn hệ số `Ki`**.

```c
ITerm = Ki * Err;      /* nhân Ki NGAY tại thời điểm cộng dồn */
Iout += ITerm;
```

Vì sao điều này quan trọng? Vì nếu bạn đổi `Ki` lúc chương trình đang chạy, cách
viết trên chỉ ảnh hưởng các bước tính về sau — lệnh ra không nhảy bậc. Còn cách
viết ngây thơ `Iout = Ki * tổng_sai_số` thì nhân lại **toàn bộ lịch sử** bằng hệ
số mới, nên vừa đổi `Ki` là cơ cấu giật một cái.

shark_pid giữ nguyên cách làm này.

### 1.2. Ba chỗ shark_pid làm khác

#### a) Bản gốc hoàn toàn không dùng `dt`

```c
pid->ITerm = pid->Ki * pid->Err;                    /* thiếu * dt */
pid->Dout  = pid->Kd * (pid->Err - pid->Last_Err);  /* thiếu / dt */
```

Struct của nó có khai báo `ControlPeriod`, `dtime`… nhưng không một dòng nào
dùng tới chúng.

**Hậu quả với người dùng:**

- `Ki` và `Kd` dính chặt vào tần số vòng lặp. Đổi từ 1 kHz sang 500 Hz là phải
  dò lại toàn bộ hệ số.
- Nếu nhịp vòng lặp bị jitter (lúc 1.0 ms lúc 1.4 ms), khâu tích phân tính sai —
  vì nó cộng dồn như thể mọi chu kỳ đều dài bằng nhau.
- Hệ số lọc `alpha` cũng cố định, nên tần số cắt của bộ lọc trôi theo tốc độ
  vòng lặp.

shark_pid nhận `dt` tường minh ở mỗi lần gọi và nhân/chia nó vào đúng chỗ, nên
không gặp cả ba vấn đề trên.

#### b) Macro `ABS` thiếu ngoặc

```c
#define ABS(x) ((x > 0) ? x : -x)
```

Đây là lỗi kinh điển của macro C. Khi bạn viết `ABS(Target - Measure)`, nhánh
"sai" nở ra thành `-Target - Measure` chứ **không phải** `-(Target - Measure)`.
Kết quả trả về không phải trị tuyệt đối.

Macro này nằm ngay trong `f_PID_ErrorHandle` — phần logic phát hiện lỗi.

shark_pid không dùng macro cho việc này. Lõi thư viện thậm chí không gọi một hàm nào
của `<math.h>`, chỉ dùng so sánh và bốn phép toán cơ bản.

#### c) Chống windup hở đúng vào lúc cần nhất

```c
if (ABS(temp_Output) > pid->MaxOut) {
    if (pid->Err * pid->Iout > 0) { pid->ITerm = 0; }
}
```

Điều kiện ở đây xét dấu của **bộ tích luỹ** `Iout`.

Vấn đề nằm ở khoảnh khắc khởi động: lúc đó `Iout = 0`, nên tích `Err * Iout`
cũng bằng 0, tức không thoả điều kiện `> 0`. Bộ chống windup không kích hoạt, và
khâu I vẫn cộng dồn thoải mái dù lệnh ra đã bão hoà từ lâu.

Nói cách khác: **cơ chế mất tác dụng đúng vào lúc windup mạnh nhất** — lúc hệ
khởi động từ xa và cơ cấu chạy hết cỡ trong thời gian dài.

shark_pid xét **lượng vượt biên có dấu** trên đúng tín hiệu mà mạch chống windup
của khối Simulink nhìn vào. Nhờ vậy nó khớp với khối kể cả ở những nhịp chạm
biên, và có thêm back-calculation làm lựa chọn mượt hơn.

> **Về `f_PID_ErrorHandle` nói chung.** Hàm phát hiện kẹt cơ cấu của bản gốc còn
> vài chỗ nữa: nó thoát sớm khi `Output` âm nên không bắt được kẹt lúc quay
> chiều ngược; nó chia cho `Target` trong khi lệnh dừng động cơ chính là
> `Target = 0`; nó đếm theo số vòng lặp chứ không theo thời gian; và cờ lỗi một
> khi bật thì chốt vĩnh viễn.
>
> shark_pid không có tính năng này, vì nó thuộc về tầng ứng dụng: bạn đếm
> xem `pid.isSaturated()` bật liên tục bao lâu trong khi giá trị đo không nhúc
> nhích.

---

## 2. SimpleFOC (Arduino-FOC)

Kho: <https://github.com/simplefoc/Arduino-FOC> — file `src/common/pid.cpp`.

Code gọn gàng, đúng đắn, và **có `dt` thật**.

Điều shark_pid học trực tiếp từ đây là cách đo `dt` bằng `micros()` sao cho đúng
kể cả khi bộ đếm bị tràn. Đó chính là hàm `SharkPID::update()` hai đối số.

> **Vì sao phép trừ vẫn đúng khi tràn số?**
>
> `micros()` trả về `unsigned long` 32-bit, và nó tràn về 0 sau khoảng 71,6
> phút. Nhưng phép trừ `timestamp_now - timestamp_prev` giữa hai số **không dấu**
> vẫn cho ra đúng khoảng cách, kể cả khi `now` đã tràn còn `prev` thì chưa. Đây
> là tính chất của số học modulo, không phải mẹo.
>
> Bộ bảo vệ `if (Ts <= 0 || Ts > 0.5f)` phục vụ chuyện khác: chặn nhịp gọi đầu
> tiên (chưa có mốc thời gian trước đó) và những lần lịch trình bị treo. Nó
> không phải để chữa tràn số.

### Những gì shark_pid thêm vào so với SimpleFOC

| | SimpleFOC | shark_pid |
|---|---|---|
| Lọc khâu D | Không có | `cfg.n`, hệ số suy lại từ `dt` mỗi chu kỳ |
| Khâu D nhìn giá trị đo | Không có — nên bị derivative kick | `cfg.c = 0` |
| Khâu P nhìn giá trị đo | Không có | `cfg.b` |
| Trần riêng cho khâu I | Dùng chung `limit` với lệnh ra | `i_min` / `i_max` tách riêng |
| Back-calculation | Không có | `SHARK_PID_F_BACKCALC_I` + `cfg.kb` |
| Chặn NaN / vô cực | Không có | Chặn ngay ở cửa vào |
| Đối chiếu được với Simulink | Không | [`extras/Test_Shark_PID/`](../extras/Test_Shark_PID/) |
| Phụ thuộc Arduino | Có (`Arduino.h`, `micros()`) | Lõi là C99 thuần |

---

## 3. Bảng tra: chuyển code từ PID_Library sang shark_pid

Nếu bạn đang có code chạy trên `WangHongxi2001/PID_Library` và muốn chuyển sang.

### 3.1. Đổi tên cờ và tham số

| Bản gốc | shark_pid |
|---|---|
| `Integral_Limit` | `cfg.i_min` / `cfg.i_max` |
| `Derivative_On_Measurement` | `cfg.c = 0.0f` *(đây là mặc định)* |
| `Trapezoid_Intergral` | `SHARK_PID_F_TRAPEZOID_I` |
| `Proportional_On_Measurement` | `cfg.b = 0.0f` *(bản gốc có khai báo cờ này nhưng chưa cài đặt)* |
| `DerivativeFilter` | `cfg.n` — chú ý: `N` tính theo rad/giây, **không phải** `alpha` |
| `OutputFilter` | **Không có** — tự lọc thông thấp sau lệnh, ở tầng gọi |
| `ChangingIntegralRate` | **Không có** |
| `ErrorHandle` | **Không có** — đếm `pid.isSaturated()` ở tầng ứng dụng |

### 3.2. Quy đổi giá trị hệ số

Vì bản gốc không nhân `dt` còn shark_pid có nhân, bạn **phải** quy đổi lại chứ
không chép thẳng con số được.

Với vòng lặp **1 kHz** (`dt = 0.001` giây):

```
Ki_mới = Ki_cũ × 1000
Kd_mới = Kd_cũ ÷ 1000
```

Công thức tổng quát cho tần số bất kỳ: `Ki_mới = Ki_cũ / dt`, `Kd_mới = Kd_cũ × dt`.

### 3.3. Quy đổi bộ lọc khâu D

Bản gốc khai bộ lọc bằng hệ số `alpha`, shark_pid khai bằng `N`:

```
N = alpha / ((1 − alpha) × dt)
```

Ví dụ: `alpha = 0.25` ở vòng lặp 1 kHz → `N = 333`, tương đương hằng số thời
gian 3 ms.

---

## 4. Vì sao thư viện không có những tính năng đó

Biến tốc độ tích phân, lọc lệnh ra, phát hiện kẹt, vùng chết, giới hạn dốc và
feedforward — **không cái nào có ô tương ứng trong khối `PID Controller (2DOF)`
của Simulink.**

Với một thư viện nhúng bình thường thì chuyện đó không sao cả. Nhưng với một thư
viện mà lời hứa là *"mô hình bạn dò trong mô phỏng chính là bộ điều khiển chạy
trên phần cứng"* thì đó lại là vấn đề lớn nhất:

**Chỉ cần bật `deadband` lên một cái, mô hình Simulink không còn tả đúng firmware
nữa.** Và thế là bạn mất khả năng kiểm chứng trước khi nạp — đúng thứ mà
[`extras/Test_Shark_PID/`](../extras/Test_Shark_PID/) sinh ra để làm.

Nên thư viện chọn: **giữ lõi đúng bằng khối, đẩy phần còn lại ra tầng gọi.** Cách
này cũng đúng với tinh thần Simulink — ở đó chúng vốn là các khối `Dead Zone`,
`Rate Limiter`, `Sum`, `Discrete Filter` nối *phía sau* khối PID, chứ không nằm
bên trong nó.

**Vậy tôi cần những tính năng đó thì làm sao?** Bảng
[Những gì thư viện cố tình không có](../README.md#9-những-gì-thư-viện-cố-tình-không-có)
trong README chỉ chỗ thay thế cho từng món. Còn
[`examples/02_Motor_Bidirectional`](../examples/02_Motor_Bidirectional) làm mẫu
món hay dùng nhất — giới hạn dốc — trong đúng 8 dòng.
