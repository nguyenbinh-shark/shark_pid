/**
 * @file    shark_pid.c
 * @brief   Cài đặt shark_pid. Xem shark_pid.h để biết API.
 * @author  Trần Nguyên Bình (github.com/nguyenbinh-shark)
 * @license MIT
 *
 * Toàn bộ hàm shark_pid_update() dưới đây là bản dịch dòng-đối-dòng của sơ đồ
 * bên trong mask khối `PID Controller (2DOF)` (Simulink R2022b). Thứ tự các
 * bước, tín hiệu mà mạch chống windup nhìn vào, và cách Discrete-Time
 * Integrator tách TRẠNG THÁI khỏi NGÕ RA đều được giữ nguyên — đó là lý do
 * hai bên khớp tới mức nhiễu số học kể cả ở những nhịp ngõ ra chạm biên.
 */
#include "shark_pid.h"
#include <stddef.h>

/* ------------------------------------------------------------------------- */
/* Tiện ích nội bộ                                                           */
/*                                                                           */
/* Toàn bộ file không gọi hàm nào của <math.h> — chỉ so sánh và bốn phép      */
/* toán. Nhờ vậy lõi biên được cho cả những toolchain nhúng tối giản, và      */
/* không phải link -lm.                                                       */
/* ------------------------------------------------------------------------- */

static float sp_clamp(float v, float lo, float hi)
{
    if (lo > hi) { float t = lo; lo = hi; hi = t; }
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/** Kiểm tra hữu hạn mà không phụ thuộc macro isfinite của toolchain. */
static int sp_finite(float v)
{
    /* v != v chỉ đúng với NaN; so sánh biên bắt ±Inf. */
    if (v != v) return 0;
    if (v > 3.402823466e38f || v < -3.402823466e38f) return 0;
    return 1;
}

/**
 * Khối `Dead Zone` trong mạch chống windup của khối PID: lượng vượt biên,
 * CÓ DẤU. Nằm trong dải thì bằng 0.
 */
static float sp_deadzone(float v, float lo, float hi)
{
    if (lo > hi) { float t = lo; lo = hi; hi = t; }
    if (v > hi) return v - hi;
    if (v < lo) return v - lo;
    return 0.0f;
}

/* ------------------------------------------------------------------------- */
/* Khởi tạo                                                                  */
/* ------------------------------------------------------------------------- */

void shark_pid_cfg_default(shark_pid_cfg_t *cfg)
{
    if (cfg == NULL) return;

    cfg->kp = 1.0f;
    cfg->ki = 0.0f;
    cfg->kd = 0.0f;

    cfg->b = 1.0f;      /* P cổ điển                         */
    cfg->c = 0.0f;      /* D-on-measurement: chặn deriv kick  */

    cfg->n = 100.0f;    /* lọc D, tương đương tau 10 ms — hầu như luôn cần */

    cfg->out_min = -100.0f;
    cfg->out_max =  100.0f;
    cfg->i_min   = -100.0f;
    cfg->i_max   =  100.0f;
    cfg->kb      = 0.0f;

    cfg->dt_max     = 0.5f;
    cfg->dt_nominal = 0.001f;

    cfg->flags = SHARK_PID_F_TRAPEZOID_I | SHARK_PID_F_CLAMP_I;
}

void shark_pid_init(shark_pid_t *pid, const shark_pid_cfg_t *cfg)
{
    if (pid == NULL) return;

    if (cfg != NULL) {
        pid->cfg = *cfg;
    } else {
        shark_pid_cfg_default(&pid->cfg);
    }
    shark_pid_reset(pid);
}

void shark_pid_reset(shark_pid_t *pid)
{
    if (pid == NULL) return;

    pid->p_term = 0.0f;
    pid->i_term = 0.0f;
    pid->d_term = 0.0f;
    pid->error = 0.0f;
    pid->output = 0.0f;
    pid->dt_used = 0.0f;
    pid->status = SHARK_PID_OK;

    pid->i_state = 0.0f;
    pid->d_state = 0.0f;
    pid->prev_d_input = 0.0f;
    pid->primed = 0u;
}

void shark_pid_set_gains(shark_pid_t *pid, float kp, float ki, float kd)
{
    if (pid == NULL) return;
    /* i_state lưu Ki*∫e (đã nhân Ki), nên đổi Ki không hồi tố lên lịch sử
       -> ngõ ra không nhảy bậc. Đây là lý do trạng thái phải chứa sẵn Ki. */
    pid->cfg.kp = kp;
    pid->cfg.ki = ki;
    pid->cfg.kd = kd;
    if (ki == 0.0f) {
        pid->i_state = 0.0f;
        pid->i_term  = 0.0f;
    }
}

void shark_pid_set_output_limits(shark_pid_t *pid, float out_min, float out_max)
{
    if (pid == NULL) return;
    if (out_min > out_max) { float t = out_min; out_min = out_max; out_max = t; }

    pid->cfg.out_min = out_min;
    pid->cfg.out_max = out_max;

    pid->output = sp_clamp(pid->output, out_min, out_max);
}

void shark_pid_preload(shark_pid_t *pid, float output_now)
{
    if (pid == NULL) return;
    if (!sp_finite(output_now)) return;

    output_now = sp_clamp(output_now, pid->cfg.out_min, pid->cfg.out_max);

    /* Nạp trạng thái bộ tích phân = ô `Integrator Initial condition`. */
    pid->i_state = sp_clamp(output_now, pid->cfg.i_min, pid->cfg.i_max);
    pid->i_term  = pid->i_state;
    pid->output  = output_now;
    pid->d_state = 0.0f;
    pid->primed  = 0u;   /* buộc lấy mốc lại -> khâu D không giật ở nhịp đầu */
}

/* ------------------------------------------------------------------------- */
/* Vòng tính chính                                                           */
/* ------------------------------------------------------------------------- */

float shark_pid_update(shark_pid_t *pid, float setpoint, float measurement, float dt)
{
    const shark_pid_cfg_t *c;
    float error, w_p, w_d;
    float p, d, kd_w;
    float u_int, pre_sat, dz;
    float i_out, i_next;
    float u, u_raw;

    if (pid == NULL) return 0.0f;
    c = &pid->cfg;

    /* --- 1. Chặn NaN/Inf ngay ở cửa ------------------------------------- */
    /* Một mẫu ADC hỏng lọt vào bộ tích phân sẽ đầu độc nó vĩnh viễn:
       i_state = NaN thì mọi chu kỳ sau đều NaN, reset mới cứu được.        */
    if (!sp_finite(setpoint) || !sp_finite(measurement)) {
        pid->status |= (uint32_t)SHARK_PID_BAD_INPUT;
        return pid->output;                 /* giữ nguyên lệnh cũ */
    }

    pid->status = SHARK_PID_OK;

    /* --- 2. Chốt dt ------------------------------------------------------ */
    /* Khối PID có `Sample time` là HẰNG SỐ nên không mô phỏng được nhịp
       jitter. Đây là phần firmware buộc phải tự lo.                        */
    if (!sp_finite(dt) || dt <= 0.0f || dt > c->dt_max) {
        pid->status |= (uint32_t)SHARK_PID_BAD_DT;
        dt = c->dt_nominal;
    }
    if (dt <= 0.0f) dt = 1e-3f;             /* dt_nominal bị cấu hình sai */
    pid->dt_used = dt;

    /* --- 3. Sai số và hai tín hiệu có trọng số -------------------------- */
    error = setpoint - measurement;
    w_p   = c->b * setpoint - measurement;  /* vào khâu P */
    w_d   = c->c * setpoint - measurement;  /* vào khâu D */
    pid->error = error;

    /* --- 4. Nhịp đầu tiên: chỉ lấy mốc lịch sử -------------------------- */
    /* Nạp trạng thái sao cho khâu D = 0 ở nhịp đầu. Đây là điểm DUY NHẤT
       shark_pid cố tình khác khối PID, và nó ánh xạ được: tương đương điền
       `Filter Initial condition` = D*(c*r0 - y0). Cho r0 = y0 thì cả hai
       cùng bằng 0 và hai bên trùng khít từ nhịp số một.                     */
    if (!pid->primed) {
        pid->prev_d_input = w_d;
        pid->d_state      = c->kd * w_d;
        pid->primed = 1u;
    }

    /* --- 5. Khâu P (có setpoint weighting b) ---------------------------- */
    /*   b = 1 -> Kp*e          (cổ điển, bám setpoint nhanh)
         b = 0 -> -Kp*y         (P-on-measurement, đổi setpoint không giật)   */
    p = c->kp * w_p;

    /* --- 6. Khâu D (có setpoint weighting c) ---------------------------- */
    /*   c = 0 -> D chỉ nhìn giá trị đo -> triệt tiêu derivative kick.
         Bộ lọc là một khâu tích phân riêng, đúng như sơ đồ của khối:
             v = N*(D*w - x_f)/(1 + N*Ts),   x_f += Ts*v
         Rời rạc hoá Backward Euler; hệ số suy TỪ dt nên đổi tần số vòng lặp
         không làm trôi tần số cắt.                                          */
    kd_w = c->kd * w_d;
    if (c->n > 0.0f) {
        d = c->n * (kd_w - pid->d_state) / (1.0f + c->n * dt);
        pid->d_state += dt * d;
    } else {
        /* Bỏ tích `Use filtered derivative`: hiệu lùi thuần D*(1 - z^-1)/Ts. */
        d = c->kd * (w_d - pid->prev_d_input) / dt;
    }

    /* --- 7. Chống windup: quyết định trên preSat CHƯA cộng nhịp này ----- */
    /* Đây là chi tiết làm nên chuyện. Mạch anti-windup của khối nhìn vào
       P + TRẠNG THÁI khâu I + D, tức chưa có phần đóng góp của nhịp hiện
       tại — Simulink buộc phải làm vậy để cắt vòng đại số, vì Backward Euler
       và Trapezoidal đều có truyền thẳng. Cài đặt "tính u_raw rồi hoàn tác
       bước tích phân" trực quan hơn nhưng KHÔNG khớp khối ở những nhịp bão
       hoà, nên ở đây theo đúng khối.                                        */
    u_int   = c->ki * error;                        /* preInt */
    pre_sat = p + pid->i_state + d;                 /* preSat */
    dz      = sp_deadzone(pre_sat, c->out_min, c->out_max);

    if (c->flags & (uint32_t)SHARK_PID_F_CLAMP_I) {
        /* clamping: ngắt tích phân khi lượng vượt biên và preInt cùng dấu. */
        if ((dz > 0.0f && u_int > 0.0f) || (dz < 0.0f && u_int < 0.0f)) {
            u_int = 0.0f;
        }
    } else if (c->flags & (uint32_t)SHARK_PID_F_BACKCALC_I) {
        /* back-calculation: kéo về theo lượng bị cắt, -dz = sat(preSat) - preSat.
           Mượt hơn kiểu kẹp vì không bật/tắt đột ngột.                       */
        u_int += c->kb * (-dz);
    }

    /* --- 8. Khâu I: đúng một nhịp của Discrete-Time Integrator ---------- */
    /* Trạng thái và ngõ ra là HAI giá trị khác nhau khi dùng hình thang, và
       cận i_min/i_max kẹp cả hai — chép nguyên cách khối làm.               */
    if (c->flags & (uint32_t)SHARK_PID_F_TRAPEZOID_I) {
        i_out  = sp_clamp(pid->i_state + 0.5f * dt * u_int, c->i_min, c->i_max);
        i_next = sp_clamp(pid->i_state + dt * u_int,        c->i_min, c->i_max);
    } else {
        i_out  = sp_clamp(pid->i_state + dt * u_int,        c->i_min, c->i_max);
        i_next = i_out;
    }
    pid->i_state = i_next;

    /* --- 9. Tổng hợp và cắt biên ---------------------------------------- */
    u_raw = p + i_out + d;
    u = sp_clamp(u_raw, c->out_min, c->out_max);
    if (u != u_raw) pid->status |= (uint32_t)SHARK_PID_SATURATED;

    /* --- 10. Lưu trạng thái cho chu kỳ sau ------------------------------- */
    pid->p_term = p;
    pid->i_term = i_out;
    pid->d_term = d;
    pid->output = u;

    pid->prev_d_input = w_d;

    return u;
}
