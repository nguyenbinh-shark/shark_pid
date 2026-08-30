# Kiểm chứng shark_pid với Simulink

Thư mục này trả lời một câu hỏi: **mã C bạn sắp nạp lên vi điều khiển có thật sự
chạy giống khối `PID Controller (2DOF)` của Simulink không?**

Không cần tin lời ai. Chạy một lệnh, đọc con số.

---

## Nó hoạt động thế nào?

Bình thường bạn có hai thứ tách rời nhau:

- Mô hình Simulink, nơi bạn dò tham số PID.
- Mã C trong firmware, nơi bạn chép tham số sang.

Bộ công cụ này ghép chúng lại trong cùng một mô hình:

```
                 ┌─────────────────────────────┐
   tín hiệu ────►│  Khối S-Function            │──► lệnh ra A
   thử           │  (gọi thẳng src/shark_pid.c)│
      │          └─────────────────────────────┘
      │                                              so sánh A và B
      │          ┌─────────────────────────────┐
      └─────────►│  Khối PID Controller (2DOF) │──► lệnh ra B
                 │  chính hãng của Simulink    │
                 └─────────────────────────────┘
```

Cùng một tín hiệu vào, cùng bộ tham số, hai đường tính toán hoàn toàn độc lập.
Nếu hai lệnh ra trùng nhau tới mức nhiễu số học, nghĩa là **bộ điều khiển trong
mô phỏng và bộ điều khiển trong firmware là một**.

> **S-Function là gì?** Là cơ chế của Simulink cho phép một khối gọi thẳng mã C
> do bạn viết, thay vì dùng khối dựng sẵn. **MEX** là định dạng thư viện biên
> dịch mà MATLAB nạp được. Ở đây, file `src/shark_pid.c` — đúng file sẽ nạp lên
> vi điều khiển, không phải bản chép lại — được biên dịch thành một khối chạy
> được trong Simulink.

---

## Cần chuẩn bị gì

- **MATLAB R2019a trở lên**, có **Simulink**.
- **Một trình biên dịch C** đã cài đặt cho MATLAB.

Kiểm tra trình biên dịch bằng lệnh sau trong cửa sổ lệnh MATLAB:

```matlab
mex -setup C
```

Nếu MATLAB báo chưa có trình biên dịch nào: trên Windows hãy cài
*MATLAB Support for MinGW-w64 C/C++ Compiler* (Add-Ons → Get Add-Ons), hoặc
Visual Studio Community. Trên Linux/macOS thì `gcc` / Xcode Command Line Tools
là đủ.

---

## Chạy thử

Trong cửa sổ lệnh của MATLAB:

```matlab
cd extras/Test_Shark_PID
verify_shark_vs_pid2
```

Chỉ vậy thôi. Script sẽ tự làm ba việc:

1. **Biên dịch** `shark_pid_sfun.c` cùng với `../../src/shark_pid.c` thành file
   MEX. Bước này chỉ chạy khi mã C mới hơn file MEX đang có, nên lần chạy thứ
   hai sẽ nhanh hơn nhiều.
2. **Dựng mô hình** `cmp_shark_pid2.slx` với hai khối chạy song song như sơ đồ ở
   trên.
3. **Chạy mô phỏng** trên tín hiệu thử tổng hợp (bậc thang + dốc + nhiễu), vẽ đồ
   thị và in ra sai số lớn nhất giữa hai bên.

### Đọc kết quả

Kết thúc, script in ra bảng tương tự:

```
=== KET QUA MO PHONG (3001 nhip, Ts = 0.001 s) ===
  Bien do max |u|max                     : 42.7
  Sai so lon nhat (C S-Function vs PID2) : 3.2e-07  (nguong 1e-05)

  >> [DAT] shark_pid.c (S-Function) KHOP HOAN TOAN voi khoi PID (2DOF) Simulink!
```

| Dòng | Nghĩa |
|---|---|
| `Bien do max` | Biên độ lớn nhất của lệnh ra, dùng làm mốc để tính sai số tương đối |
| `Sai so lon nhat` | Chênh lệch lớn nhất giữa hai lệnh ra trong suốt mô phỏng |
| `nguong 1e-05` | Ngưỡng đạt |
| `[DAT]` / `[LECH]` | Kết luận |

**Vì sao ngưỡng không phải là 0?** Vì mã C dùng kiểu `float` (32 bit) còn Simulink
tính bằng `double` (64 bit). Hai kiểu số này làm tròn khác nhau, và khoảng
`1e-7`…`1e-6` là sàn không thể vượt qua. Ngưỡng `1e-5` đặt ngay trên cái sàn đó.

---

## Thử với tham số của riêng bạn

Truyền tham số theo từng cặp `'tên', giá trị`:

```matlab
verify_shark_vs_pid2('kp', 4, 'ki', 1.5, 'kd', 0.1, 'n', 250, 'Ts', 0.001);
```

Vài tình huống hay dùng:

```matlab
% Kiểm tra lúc lệnh ra bị bão hoà — đây là chỗ đáng nghi nhất
verify_shark_vs_pid2('out_min', -10, 'out_max', 10, 'AntiWindup', 'clamping');

% Chống windup kiểu back-calculation
verify_shark_vs_pid2('out_min', -10, 'out_max', 10, ...
                     'AntiWindup', 'back-calculation', 'kb', 2.0);

% Đổi phương pháp tích phân
verify_shark_vs_pid2('IntegratorMethod', 'Backward Euler');

% Vòng lặp chậm 50 Hz, chạy 10 giây
verify_shark_vs_pid2('Ts', 0.02, 'Tend', 10);
```

### Danh sách tham số

**Tham số PID** — tên trùng với trường trong `shark_pid_cfg_t`:

| Tên | Mặc định trong script | Ý nghĩa |
|---|---|---|
| `kp`, `ki`, `kd` | `4.0`, `1.5`, `0.10` | Ba hệ số PID |
| `b`, `c` | `0.80`, `0.00` | Trọng số setpoint cho khâu P và khâu D |
| `n` | `50.0` | Hệ số lọc khâu D |
| `out_min`, `out_max` | `-1e6`, `1e6` | Giới hạn lệnh ra. Để rộng như mặc định nghĩa là không bao giờ bão hoà |
| `i_min`, `i_max` | `-1e6`, `1e6` | Giới hạn khâu I |
| `kb` | `0.0` | Hệ số back-calculation |

**Tuỳ chọn chạy:**

| Tên | Mặc định | Ý nghĩa |
|---|---|---|
| `Ts` | `1e-3` | Chu kỳ lấy mẫu, giây |
| `Tend` | `3.0` | Thời gian mô phỏng, giây |
| `IntegratorMethod` | `'Trapezoidal'` | Hoặc `'Backward Euler'` |
| `AntiWindup` | `'clamping'` | Hoặc `'back-calculation'`, hoặc `'none'` |
| `Plot` | `true` | Có vẽ đồ thị hay không |
| `Rebuild` | `false` | Đặt `true` để ép biên dịch lại file MEX |
| `Keep` | `true` | Giữ mô hình lại sau khi chạy xong |
| `Verbose` | `true` | In chi tiết cấu hình trước khi chạy |

> **Mẹo:** muốn kiểm tra kỹ, hãy thử các cấu hình **có bão hoà** (`out_min` /
> `out_max` hẹp lại). Đó chính là chỗ mà phần lớn thư viện PID khác lệch khỏi
> khối Simulink, và cũng là chỗ nguy hiểm nhất khi chạy thật.

---

## Video hướng dẫn thao tác

[![Hướng dẫn kiểm chứng Shark_PID trên Simulink](https://img.youtube.com/vi/Ys2CLVu3MWg/0.jpg)](https://youtu.be/Ys2CLVu3MWg)

---

## Dùng khối shark_pid trong mô hình của riêng bạn

Ngoài việc đối chiếu, bạn có thể lấy khối S-Function này đặt vào mô hình của
mình — để mô phỏng chính xác firmware trước khi nạp.

Lệnh dưới đây tạo một khối có hộp thoại tham số thân thiện, thay vì phải gõ một
chuỗi tên biến dài:

```matlab
make_shark_mask()                          % tạo một model thư viện riêng chứa khối
make_shark_mask('cmp_shark_pid2/SHARK')    % hoặc gắn mask cho khối đã có sẵn
```

---

## Các file trong thư mục

| File | Vai trò |
|---|---|
| [`verify_shark_vs_pid2.m`](verify_shark_vs_pid2.m) | **Script chính.** Dựng mô hình, chạy đối chiếu, vẽ đồ thị, báo cáo sai số |
| [`shark_pid_sfun.c`](shark_pid_sfun.c) | Khối S-Function bọc thẳng `../../src/shark_pid.c`. Tham số cố định |
| [`shark_pid_sfun_tunable.c`](shark_pid_sfun_tunable.c) | Bản S-Function nhận hệ số qua cổng tín hiệu vào, dùng khi cần quét tham số lúc đang chạy |
| [`shark_pid_sfun_common.h`](shark_pid_sfun_common.h) | Phần dùng chung giữa hai file S-Function trên |
| [`make_shark_mask.m`](make_shark_mask.m) | Tạo hộp thoại tham số (mask) cho khối S-Function |
| `cmp_shark_pid2.slx` | Mô hình đối chiếu. Script tự dựng lại nếu chưa có |
| `dothi.jpg` | Ảnh đồ thị kết quả mẫu |

---

## Bảng ánh xạ: code C ↔ ô trong khối Simulink

Đây là bảng bạn cần khi chép tham số qua lại. Khối phải đặt ở chế độ
`Form = Parallel`, `Time domain = Discrete-time`.

| Trong code (`shark_pid_cfg_t`) | Ô trong Block Parameters |
|---|---|
| `dt` *(đối số của `shark_pid_update`)* | `Sample time` |
| `kp`, `ki`, `kd` | `P`, `I`, `D` |
| `b`, `c` | `Setpoint weight (b)`, `Setpoint weight (c)` |
| `n > 0` | Tích `Use filtered derivative`, `Filter coefficient (N)` = `n`, `Filter method = Backward Euler` |
| `n <= 0` | Bỏ tích `Use filtered derivative` |
| Cờ `SHARK_PID_F_TRAPEZOID_I` bật / tắt | `Integrator method` = `Trapezoidal` / `Backward Euler` |
| `out_min`, `out_max` | Tích `Limit output`, điền `Lower/Upper saturation limit` |
| `i_min`, `i_max` | Tích `Limit integrator`, điền `Lower/Upper integrator saturation limit` |
| Cờ `SHARK_PID_F_CLAMP_I` | `Anti-windup method` = `clamping` |
| Cờ `SHARK_PID_F_BACKCALC_I` + `kb` | `Anti-windup method` = `back-calculation`, `Kb` = `kb` |
| `shark_pid_preload()` | `Integrator Initial condition` trong tab `Initialization` |
| `dt_max`, `dt_nominal` | *Không có ô tương ứng* — lưới an toàn chỉ firmware mới cần |

---

## Gặp lỗi thì xem ở đây

| Thông báo / hiện tượng | Nguyên nhân | Cách xử lý |
|---|---|---|
| `No supported compiler was found` | MATLAB chưa có trình biên dịch C | Chạy `mex -setup C`; cài MinGW-w64 qua Add-Ons |
| Lỗi biên dịch nhắc tới `shark_pid.c` | Sai đường dẫn, hoặc bạn vừa sửa mã C và nó không biên được | Đảm bảo đang ở đúng thư mục `extras/Test_Shark_PID`; biên thử mã C bằng `gcc` trước |
| `Undefined function 'verify_shark_vs_pid2'` | MATLAB chưa ở đúng thư mục | `cd extras/Test_Shark_PID` rồi chạy lại |
| Kết quả `[LECH]` sau khi bạn sửa mã C | Thay đổi của bạn đã làm lệch khỏi khối Simulink | Xem đồ thị: lệch ở nhịp nào, có phải nhịp bão hoà không |
| Kết quả `[LECH]` mà bạn chưa sửa gì | File MEX cũ, không khớp mã C hiện tại | Chạy `verify_shark_vs_pid2('Rebuild', true)` |
| MATLAB báo khối bị khoá hoặc mô hình mở sẵn | Mô hình từ lần chạy trước còn mở | `close_system('cmp_shark_pid2', 0)` rồi chạy lại |
| `Khong co tuy chon hoac tham so ten "..."` | Gõ sai tên tham số | Đối chiếu với hai bảng tham số ở trên |

---

Quay lại [README chính](../../README.md).
