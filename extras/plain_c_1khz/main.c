/**
* plain_c_1khz — Dùng shark_pid từ C thuần, không Arduino.
 *
 * Kịch bản: một khớp cánh tay robot chạy trong ngắt timer 1 kHz trên STM32.
 * Minh hoạ hai thứ mà bản Arduino không thể hiện rõ:
 *
 *   1. dt tường minh, lấy từ chu kỳ timer — không phải đo lại mỗi vòng.
 *   2. Feedforward bù trọng lực: mô-men giữ cánh tay phụ thuộc góc khớp,
 *      không suy ra được từ setpoint nên phải truyền từ ngoài vào bằng
 *      shark_pid_update_ff().
 *
 * Điểm đáng quan sát trong kết quả chạy: ở trạng thái xác lập, feedforward
 * gánh toàn bộ tải tĩnh (12.73 = 18·cos45°) nên khâu I tiến về 0. Không có
 * feedforward thì chính khâu I phải è cổ giữ con số 12.73 đó — chậm hơn và
 * ăn hết dải chống nhiễu của bộ tích phân.
 *
 * Biên dịch và chạy thử trên PC:
 *   gcc -std=c99 -Wall -Wextra -I../../src ../../src/shark_pid.c main.c -lm -o demo
 */
#include "shark_pid.h"
#include <math.h>
#include <stdio.h>

#define CTRL_HZ   1000.0f
#define CTRL_DT   (1.0f / CTRL_HZ)

#define DEG2RAD   0.017453292f

/* Mô-men cần để giữ cánh tay nằm ngang, đơn vị % công suất. */
#define ARM_HOLD_TORQUE  18.0f

static shark_pid_t joint;

void joint_control_init(void)
{
    shark_pid_cfg_t cfg;
    shark_pid_cfg_default(&cfg);

    cfg.kp = 4.0f;
    cfg.ki = 1.5f;      /* đơn vị 1/giây */
    cfg.kd = 0.1f;      /* đơn vị giây   */

    cfg.b = 1.0f;       /* xem ghi chú về b < 1 trong README trước khi hạ xuống */
    cfg.c = 0.0f;       /* D chỉ nhìn encoder -> hết derivative kick */

    /* cfg.kf_dot: chỉ có tác dụng khi setpoint ĐANG CHẠY (bám quỹ đạo).
       Demo này giữ setpoint cố định nên để 0 cho khỏi gây hiểu nhầm. */
    cfg.kf_dot = 0.0f;

    cfg.out_min = -100.0f;
    cfg.out_max =  100.0f;
    cfg.i_min   =  -60.0f;
    cfg.i_max   =   60.0f;

    cfg.d_tau    = 0.004f;    /* lọc D 4 ms */
    cfg.out_slew = 800.0f;    /* %/giây — bảo vệ hộp số khỏi sốc dòng */

    cfg.dt_nominal = CTRL_DT;
    cfg.dt_max     = 0.05f;   /* quá 50 ms nghĩa là lịch trình đã hỏng */

    /* Back-calculation cho đáp ứng mượt hơn kiểu kẹp cứng.
       kt lớn thì chống windup mạnh hơn; khởi điểm ~ ki/kp rồi tăng dần. */
    cfg.kt = 3.0f;
    cfg.flags = SHARK_PID_F_TRAPEZOID_I | SHARK_PID_F_BACKCALC_I;

    shark_pid_init(&joint, &cfg);
}

/* Gọi trong ngắt timer 1 kHz. */
float joint_control_step(float target_deg, float measured_deg)
{
    /* Bù trọng lực tính từ góc ĐO ĐƯỢC, không phải từ setpoint. */
    float gravity_ff = ARM_HOLD_TORQUE * cosf(measured_deg * DEG2RAD);

    return shark_pid_update_ff(&joint, target_deg, measured_deg, CTRL_DT, gravity_ff);
}

/* --------------------------------------------------------------------- */
/* Mô phỏng nhỏ để chạy thử ngay trên PC.                                */
/* Khớp dùng driver chế độ tốc độ (rất phổ biến với động cơ có hộp số):  */
/* mô-men dư sau khi trừ trọng lực sinh ra tốc độ quay.                  */
/* --------------------------------------------------------------------- */
#define JOINT_GAIN  0.5f          /* (độ/giây) trên mỗi đơn vị mô-men dư */

int main(void)
{
    float angle = 0.0f;
    const float target = 45.0f;
    int i;

    joint_control_init();

    printf("  t(s)     goc     lenh       P       I       D      FF\n");

    for (i = 0; i < 10000; ++i) {           /* 10 giây */
        float u = joint_control_step(target, angle);

        float excess = u - ARM_HOLD_TORQUE * cosf(angle * DEG2RAD);
        angle += JOINT_GAIN * excess * CTRL_DT;

        if (i % 1000 == 0) {
            /* printf luôn thăng float lên double, nên cast tường minh cho sạch. */
            printf("%6.2f %7.3f %8.2f %7.2f %7.2f %7.2f %7.2f\n",
                   (double)(i * CTRL_DT), (double)angle, (double)u,
                   (double)joint.p_term, (double)joint.i_term,
                   (double)joint.d_term, (double)joint.ff_term);
        }
    }

    printf("\nGoc cuoi = %.5f  (dat = %.1f)  sai so = %.5f\n",
           (double)angle, (double)target, (double)(target - angle));
    printf("Feedforward giu tai tinh = %.2f, nen khau I ve gan 0 (%.2f).\n",
           (double)joint.ff_term, (double)joint.i_term);
    return 0;
}
