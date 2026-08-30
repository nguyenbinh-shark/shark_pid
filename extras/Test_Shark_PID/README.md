# `extras/Test_Shark_PID/`

Bộ công cụ đối chiếu **mã C thật (`src/shark_pid.c`)** với khối **`PID Controller (2DOF)`** chuẩn của MATLAB/Simulink thông qua cơ chế **C MEX S-Function**.

Simulink sẽ gọi trực tiếp mã C sẽ nạp lên vi điều khiển để kiểm chứng rằng: **bộ điều khiển chạy trên phần cứng và mô hình bạn dò trong mô phỏng là một.**

---

## Hướng dẫn chạy

Trong MATLAB Command Window:

```matlab
cd extras/Test_Shark_PID
verify_shark_vs_pid2                    % Tự động biên dịch MEX nếu cần và chạy mô phỏng
```

Script sẽ:
1. Tự động biên dịch `shark_pid_sfun.c` kết hợp với `../../src/shark_pid.c` thành file MEX (nếu mã nguồn C có thay đổi).
2. Tự động dựng mô hình Simulink `cmp_shark_pid2.slx` chạy song song hai khối:
   - Khối **S-Function** bọc mã C thật `shark_pid.c`.
   - Khối **PID Controller (2DOF)** chuẩn của Simulink.
3. Chạy mô phỏng trên tín hiệu thử kết hợp (Step + Ramp + Noise) và xuất đồ thị + sai số tuyệt đối.

---

## Video Hướng Dẫn Thao Tác

[![Hướng dẫn kiểm chứng Shark_PID trên Simulink](https://img.youtube.com/vi/YOUR_VIDEO_ID/0.jpg)](https://www.youtube.com/watch?v=YOUR_VIDEO_ID)
*(Thay `YOUR_VIDEO_ID` bằng ID video YouTube của bạn)*

---

## Các file trong thư mục

| File | Vai trò |
|---|---|
| [`verify_shark_vs_pid2.m`](verify_shark_vs_pid2.m) | Script chính: dựng mô hình, chạy đối chiếu S-function vs khối PID (2DOF), vẽ đồ thị và báo cáo sai số. |
| [`shark_pid_sfun.c`](shark_pid_sfun.c) | Level-2 C MEX S-function bọc trực tiếp `../../src/shark_pid.c`. |
| [`shark_pid_sfun_tunable.c`](shark_pid_sfun_tunable.c) | Bản S-function với hệ số là cổng tín hiệu vào động (dùng khi quét tham số trong lúc chạy). |
| [`shark_pid_sfun_common.h`](shark_pid_sfun_common.h) | Tiện ích C dùng chung giữa các file S-function. |
| [`make_shark_mask.m`](make_shark_mask.m) | Tạo mask giao diện hộp thoại thân thiện cho khối S-function trong Simulink. |

---

## Yêu cầu môi trường

- MATLAB R2019a trở lên kèm **Simulink**.
- Một trình biên dịch C đã thiết lập (`mex -setup C`).
