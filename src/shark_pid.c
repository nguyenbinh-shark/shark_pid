/**
 * @file    shark_pid.c
 * @brief   Cài đặt shark_pid. Xem shark_pid.h để biết API.
 * @author  Trần Nguyên Bình (github.com/nguyenbinh-shark)
 * @license MIT
 */
#include "shark_pid.h"
#include <math.h>
#include <stddef.h>

/* ------------------------------------------------------------------------- */
/* Tiện ích nội bộ                                                           */
/*                                                                           */
/* Không dùng macro ABS(x) kiểu ((x>0)?x:-x): thiếu ngoặc thì ABS(a-b) nở ra  */
/* thành (-a-b) — một lỗi kinh điển. Ở đây dùng fabsf và static inline.       */
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
 * Lọc thông thấp bậc một, hệ số suy ra TỪ dt.
 *
 *   alpha = dt / (tau + dt)
 *
 * Đây là điểm khác cốt lõi so với các thư viện dùng alpha cố định: tần số cắt
 * ở đây bám theo hằng số thời gian tau (giây) nên đổi tần số vòng lặp không
 * làm trôi đặc tính lọc.
 */
static float sp_lpf(float prev, float x, float tau, float dt)
{
    float alpha;
    if (tau <= 0.0f) return x;          /* tau = 0 -> tắt lọc */
    alpha = dt / (tau + dt);            /* tau > 0 và dt > 0 -> mẫu số luôn > 0 */
    return prev + alpha * (x - prev);
}

/**
 * Hệ số biến tốc độ tích phân f(|e|) ∈ [0, 1].
 *
 *   |e| <= b        -> 1            (áp sát đích: mở hết tích phân)
 *   b < |e| < a + b -> (a+b-|e|)/a  (chuyển tiếp tuyến tính)
 *   |e| >= a + b    -> 0            (còn xa: tắt tích phân, nhường khâu P)
 */
static float sp_ci_factor(float abs_e, float a, float b)
{
    if (a <= 0.0f) return 1.0f;         /* a = 0 -> tắt tính năng */
    if (b < 0.0f) b = 0.0f;
    if (abs_e <= b) return 1.0f;
    if (abs_e >= a + b) return 0.0f;
    return (a + b - abs_e) / a;
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

    cfg->b = 1.0f;      /* P cổ điển                       */
    cfg->c = 0.0f;      /* D-on-measurement: chặn deriv kick */

    cfg->kf = 0.0f;
    cfg->kf_dot = 0.0f;

    cfg->out_min = -100.0f;
    cfg->out_max =  100.0f;
    cfg->i_min   = -100.0f;
    cfg->i_max   =  100.0f;
    cfg->kt      = 0.0f;

    cfg->deadband = 0.0f;

    cfg->ci_a = 0.0f;
    cfg->ci_b = 0.0f;

    cfg->d_tau   = 0.01f;   /* lọc D 10 ms — hầu như luôn cần */
    cfg->out_tau = 0.0f;    /* lọc ngõ ra: mặc định TẮT, vì nó ăn vào biên độ pha */
    cfg->out_slew = 0.0f;

    cfg->dt_max     = 0.5f;
    cfg->dt_nominal = 0.001f;

    cfg->stall_time  = 0.0f;
    cfg->stall_level = 0.95f;
    cfg->stall_eps   = 0.0f;

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
    pid->ff_term = 0.0f;
    pid->error = 0.0f;
    pid->output = 0.0f;
    pid->dt_used = 0.0f;
    pid->status = SHARK_PID_OK;

    pid->prev_error = 0.0f;
    pid->prev_d_input = 0.0f;
    pid->prev_setpoint = 0.0f;
    pid->prev_meas = 0.0f;
    pid->prev_output = 0.0f;
    pid->d_filt = 0.0f;
    pid->out_filt = 0.0f;
    pid->stall_timer = 0.0f;
    pid->primed = 0u;
}

void shark_pid_set_gains(shark_pid_t *pid, float kp, float ki, float kd)
{
    if (pid == NULL) return;
    /* i_term lưu Ki*∫e (đã nhân Ki), nên đổi Ki không hồi tố lên lịch sử
       -> ngõ ra không nhảy bậc. Đây là lý do accumulator phải chứa sẵn Ki. */
    pid->cfg.kp = kp;
    pid->cfg.ki = ki;
    pid->cfg.kd = kd;
    if (ki == 0.0f) pid->i_term = 0.0f;
}

void shark_pid_set_output_limits(shark_pid_t *pid, float out_min, float out_max)
{
    if (pid == NULL) return;
    if (out_min > out_max) { float t = out_min; out_min = out_max; out_max = t; }

    pid->cfg.out_min = out_min;
    pid->cfg.out_max = out_max;

    pid->output      = sp_clamp(pid->output, out_min, out_max);
    pid->prev_output = sp_clamp(pid->prev_output, out_min, out_max);
    pid->out_filt    = sp_clamp(pid->out_filt, out_min, out_max);
}

void shark_pid_preload(shark_pid_t *pid, float output_now)
{
    if (pid == NULL) return;
    if (!sp_finite(output_now)) return;

    output_now = sp_clamp(output_now, pid->cfg.out_min, pid->cfg.out_max);

    pid->i_term      = sp_clamp(output_now, pid->cfg.i_min, pid->cfg.i_max);
    pid->output      = output_now;
    pid->prev_output = output_now;
    pid->out_filt    = output_now;
    pid->d_filt      = 0.0f;
    pid->primed      = 0u;   /* buộc lấy mốc lại -> khâu D không giật ở nhịp đầu */
}

void shark_pid_clear_status(shark_pid_t *pid)
{
    if (pid == NULL) return;
    pid->status = SHARK_PID_OK;
    pid->stall_timer = 0.0f;
}

/* ------------------------------------------------------------------------- */
/* Vòng tính chính                                                           */
/* ------------------------------------------------------------------------- */

float shark_pid_update(shark_pid_t *pid, float setpoint, float measurement, float dt)
{
    return shark_pid_update_ff(pid, setpoint, measurement, dt, 0.0f);
}

float shark_pid_update_ff(shark_pid_t *pid, float setpoint, float measurement,
                          float dt, float ff_extra)
{
    const shark_pid_cfg_t *c;
    float error, abs_e, e_int;
    float p, d, ff, d_input, d_raw;
    float i_step, i_before, area, factor;
    float u, u_raw;
    int in_deadband;

    if (pid == NULL) return 0.0f;
    c = &pid->cfg;

    /* --- 1. Chặn NaN/Inf ngay ở cửa ------------------------------------- */
    /* Một mẫu ADC hỏng lọt vào bộ tích phân sẽ đầu độc nó vĩnh viễn:
       i_term = NaN thì mọi chu kỳ sau đều NaN, reset mới cứu được.         */
    if (!sp_finite(setpoint) || !sp_finite(measurement) || !sp_finite(ff_extra)) {
        pid->status |= (uint32_t)SHARK_PID_BAD_INPUT;
        return pid->output;                 /* giữ nguyên lệnh cũ */
    }

    /* Xoá các cờ tức thời; SHARK_PID_STALLED được giữ lại (phải xoá thủ công). */
    pid->status &= ~(uint32_t)(SHARK_PID_BAD_INPUT | SHARK_PID_BAD_DT | SHARK_PID_SATURATED);

    /* --- 2. Chốt dt ------------------------------------------------------ */
    if (!sp_finite(dt) || dt <= 0.0f || dt > c->dt_max) {
        pid->status |= (uint32_t)SHARK_PID_BAD_DT;
        dt = c->dt_nominal;
    }
    if (dt <= 0.0f) dt = 1e-3f;             /* dt_nominal bị cấu hình sai */
    pid->dt_used = dt;

    /* --- 3. Sai số và vùng chết ----------------------------------------- */
    error = setpoint - measurement;
    abs_e = fabsf(error);
    pid->error = error;

    in_deadband = (c->deadband > 0.0f && abs_e < c->deadband);
    e_int = in_deadband ? 0.0f : error;

    /* --- 4. Nhịp đầu tiên: chỉ lấy mốc lịch sử -------------------------- */
    /* Không trả về 0 như nhiều thư viện khác — vẫn xuất P + I + FF hợp lệ,
       chỉ riêng khâu D bằng 0 vì chưa có mốc trước để lấy hiệu.            */
    if (!pid->primed) {
        pid->prev_error    = e_int;
        pid->prev_setpoint = setpoint;
        pid->prev_meas     = measurement;
        pid->prev_d_input  = c->c * setpoint - measurement;
        pid->primed = 1u;
    }

    /* --- 5. Khâu P (có setpoint weighting b) ---------------------------- */
    /*   b = 1 -> Kp*e          (cổ điển, bám setpoint nhanh)
         b = 0 -> -Kp*y         (P-on-measurement, đổi setpoint không giật)   */
    p = in_deadband ? 0.0f : c->kp * (c->b * setpoint - measurement);

    /* --- 6. Khâu D (có setpoint weighting c) ---------------------------- */
    /*   c = 0 -> D chỉ nhìn giá trị đo -> triệt tiêu derivative kick.
         Khâu D VẪN chạy trong vùng chết: vật đang trôi thì vẫn phải hãm.    */
    d_input = c->c * setpoint - measurement;
    d_raw   = c->kd * (d_input - pid->prev_d_input) / dt;
    pid->d_filt = sp_lpf(pid->d_filt, d_raw, c->d_tau, dt);
    d = pid->d_filt;

    /* --- 7. Feedforward -------------------------------------------------- */
    ff = c->kf * setpoint + ff_extra;
    if (c->kf_dot != 0.0f) {
        ff += c->kf_dot * (setpoint - pid->prev_setpoint) / dt;
    }

    /* --- 8. Khâu I ------------------------------------------------------- */
    i_before = pid->i_term;
    i_step = 0.0f;
    if (!in_deadband && c->ki != 0.0f) {
        /* Tích phân hình thang (Tustin) hay hình chữ nhật */
        area = (c->flags & (uint32_t)SHARK_PID_F_TRAPEZOID_I)
             ? 0.5f * (e_int + pid->prev_error)
             : e_int;
        factor = sp_ci_factor(abs_e, c->ci_a, c->ci_b);
        i_step = c->ki * factor * area * dt;
        pid->i_term = sp_clamp(pid->i_term + i_step, c->i_min, c->i_max);
    }

    /* --- 9. Tổng hợp và cắt biên ---------------------------------------- */
    u_raw = p + pid->i_term + d + ff;
    u = sp_clamp(u_raw, c->out_min, c->out_max);
    if (u != u_raw) pid->status |= (uint32_t)SHARK_PID_SATURATED;

    /* --- 10. Chống windup ------------------------------------------------ */
    /* Kẹp có điều kiện: chỉ hoàn tác khi bước tích phân vừa rồi đẩy SÂU THÊM
       vào vùng bão hoà. Điều kiện dựa trên dấu của (u_raw - u), tức trạng
       thái bão hoà thực của ngõ ra — không dựa vào dấu của bộ tích luỹ, vốn
       bằng 0 lúc khởi động và làm cơ chế mất tác dụng đúng lúc cần nhất.    */
    if ((c->flags & (uint32_t)SHARK_PID_F_CLAMP_I) && (u != u_raw) && (i_step != 0.0f)) {
        if ((u_raw > u && i_step > 0.0f) || (u_raw < u && i_step < 0.0f)) {
            pid->i_term = i_before;
            u_raw = p + pid->i_term + d + ff;
            u = sp_clamp(u_raw, c->out_min, c->out_max);
        }
    }

    /* Back-calculation: kéo tích phân về tỉ lệ với lượng bị cắt. Mượt hơn
       kiểu kẹp vì không bật/tắt đột ngột. Dùng riêng hoặc kèm kẹp đều được. */
    if ((c->flags & (uint32_t)SHARK_PID_F_BACKCALC_I) && c->kt > 0.0f) {
        pid->i_term = sp_clamp(pid->i_term + c->kt * (u - u_raw) * dt,
                               c->i_min, c->i_max);
    }

    /* --- 11. Lọc ngõ ra --------------------------------------------------- */
    pid->out_filt = sp_lpf(pid->out_filt, u, c->out_tau, dt);
    u = pid->out_filt;

    /* --- 12. Giới hạn dốc du/dt ------------------------------------------ */
    if (c->out_slew > 0.0f) {
        float max_step = c->out_slew * dt;
        float delta = u - pid->prev_output;
        if (delta > max_step)       u = pid->prev_output + max_step;
        else if (delta < -max_step) u = pid->prev_output - max_step;
        pid->out_filt = u;          /* giữ bộ lọc đồng bộ với lệnh thực xuất */
    }

    /* --- 13. Vùng chết kiểu "đóng băng lệnh" ----------------------------- */
    if (in_deadband && (c->flags & (uint32_t)SHARK_PID_F_DEADBAND_HOLD)) {
        u = pid->prev_output;
        pid->out_filt = u;
    }

    u = sp_clamp(u, c->out_min, c->out_max);

    /* --- 14. Phát hiện kẹt cơ cấu ---------------------------------------- */
    /* Đếm bằng THỜI GIAN chứ không bằng số vòng lặp, và dùng trị tuyệt đối
       nên phát hiện được cả khi động cơ chạy chiều âm.                      */
    if (c->stall_time > 0.0f) {
        float bound = (u >= 0.0f) ? c->out_max : c->out_min;
        float speed = fabsf(measurement - pid->prev_meas) / dt;
        float lvl = sp_clamp(c->stall_level, 0.0f, 1.0f);

        if (fabsf(bound) > 0.0f && fabsf(u) >= fabsf(bound) * lvl && speed < c->stall_eps) {
            pid->stall_timer += dt;
            if (pid->stall_timer >= c->stall_time) {
                pid->status |= (uint32_t)SHARK_PID_STALLED;
            }
        } else {
            pid->stall_timer = 0.0f;
        }
    }

    /* Mặc định chỉ BÁO, không tự ngắt: ngắt công suất im lặng là hành vi
       gây bất ngờ. Bật SHARK_PID_F_STALL_CUTOFF nếu muốn tự bảo vệ.        */
    if ((pid->status & (uint32_t)SHARK_PID_STALLED) &&
        (c->flags & (uint32_t)SHARK_PID_F_STALL_CUTOFF)) {
        u = sp_clamp(0.0f, c->out_min, c->out_max);
    }

    /* --- 15. Lưu trạng thái cho chu kỳ sau -------------------------------- */
    pid->p_term  = p;
    pid->d_term  = d;
    pid->ff_term = ff;
    pid->output  = u;

    pid->prev_error    = e_int;
    pid->prev_d_input  = d_input;
    pid->prev_setpoint = setpoint;
    pid->prev_meas     = measurement;
    pid->prev_output   = u;

    return u;
}
