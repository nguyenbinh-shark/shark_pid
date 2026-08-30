/**
 * @file    test_shark_pid.c
 * @brief   Bộ test shark_pid — chạy trên máy tính, không cần vi điều
 *          khiển và không phụ thuộc thư viện test bên ngoài.
 *          Trả về 0 nếu tất cả đạt, 1 nếu có bài hỏng.
 *
 * Biên dịch và chạy:
 *   cc -std=c99 -O2 -Wall -Wextra -I../src ../src/shark_pid.c test_shark_pid.c -lm -o t
 *   ./t
 *
 * Hoặc:  make test        (và  make asan  để chạy lại dưới ASan + UBSan)
 *
 * Nhóm 0 là nhóm quan trọng nhất: nó dựng lại phương trình sai phân của khối
 * `PID Controller (2DOF)` bằng kiểu double, độc lập với lõi, rồi so từng nhịp.
 * Đây là bản chạy trên PC của phép đối chiếu mà extras/Test_Shark_PID/ làm
 * trong Simulink — chạy được ở mọi máy CI, không cần MATLAB.
 */
#include "shark_pid.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/* ========================================================================= */
/* Khung test tối giản                                                       */
/* ========================================================================= */

static int g_pass = 0;
static int g_fail = 0;
static const char *g_section = "";

static void section(const char *name)
{
    g_section = name;
    printf("\n== %s ==\n", name);
}

static void check(int cond, const char *what, const char *detail)
{
    if (cond) {
        g_pass++;
        printf("  dat   %s%s%s\n", what, detail ? "   " : "", detail ? detail : "");
    } else {
        g_fail++;
        printf("  HONG  [%s] %s%s%s\n", g_section, what,
               detail ? "   " : "", detail ? detail : "");
    }
}

static const char *fmt(const char *f, double a, double b)
{
    static char buf[192];
    snprintf(buf, sizeof buf, f, a, b);
    return buf;
}

/* Số giả ngẫu nhiên tất định: cùng chuỗi trên mọi máy, mọi trình biên dịch. */
static uint32_t g_rng = 2463534242u;

static void rng_reset(void)
{
    g_rng = 2463534242u;
}

static double frand_pm1(void)
{
    g_rng = g_rng * 1664525u + 1013904223u;
    return (double)(g_rng >> 8) / 8388608.0 - 1.0;   /* [-1, 1) */
}

/* ========================================================================= */
/* NHÓM 0 — Mô hình đối chiếu: phương trình sai phân của khối PID (2DOF)     */
/*                                                                           */
/* Viết lại bằng double theo đúng sơ đồ khối ghi trong shark_pid.h:          */
/*                                                                           */
/*   u = P*(b*r - y) + I*Fi(z)*(r - y) + Dbranch(z)*(c*r - y)                */
/*                                                                           */
/* Mạch chống windup quyết định trên preSat = P + TRẠNG THÁI khâu I + D,     */
/* tức chưa cộng đóng góp của nhịp hiện tại — đúng như Simulink buộc phải    */
/* làm để cắt vòng đại số. Bộ tích phân tách TRẠNG THÁI khỏi NGÕ RA và kẹp   */
/* riêng từng cái, đúng như khối Discrete-Time Integrator.                   */
/* ========================================================================= */

typedef struct {
    double kp, ki, kd, b, c, n;
    double out_min, out_max, i_min, i_max, kb;
    int trapezoid, clamp_i, backcalc_i;

    double i_state, d_state, prev_d_input;
    int primed;
} ref_pid_t;

static double ref_clamp(double v, double lo, double hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* Khối `Dead Zone` trong mạch chống windup: lượng vượt biên, CÓ DẤU. */
static double ref_deadzone(double v, double lo, double hi)
{
    if (v > hi) return v - hi;
    if (v < lo) return v - lo;
    return 0.0;
}

static void ref_init(ref_pid_t *r, const shark_pid_cfg_t *c)
{
    memset(r, 0, sizeof *r);
    r->kp = (double)c->kp;
    r->ki = (double)c->ki;
    r->kd = (double)c->kd;
    r->b  = (double)c->b;
    r->c  = (double)c->c;
    r->n  = (double)c->n;
    r->out_min = (double)c->out_min;
    r->out_max = (double)c->out_max;
    r->i_min   = (double)c->i_min;
    r->i_max   = (double)c->i_max;
    r->kb      = (double)c->kb;
    r->trapezoid  = (c->flags & (uint32_t)SHARK_PID_F_TRAPEZOID_I) != 0u;
    r->clamp_i    = (c->flags & (uint32_t)SHARK_PID_F_CLAMP_I) != 0u;
    r->backcalc_i = (c->flags & (uint32_t)SHARK_PID_F_BACKCALC_I) != 0u;
}

static double ref_update(ref_pid_t *r, double sp, double meas, double dt)
{
    double error = sp - meas;
    double w_p   = r->b * sp - meas;
    double w_d   = r->c * sp - meas;
    double p, d, u_int, pre_sat, dz, i_out, i_next;

    if (!r->primed) {
        r->prev_d_input = w_d;
        r->d_state      = r->kd * w_d;
        r->primed = 1;
    }

    p = r->kp * w_p;

    if (r->n > 0.0) {
        d = r->n * (r->kd * w_d - r->d_state) / (1.0 + r->n * dt);
        r->d_state += dt * d;
    } else {
        d = r->kd * (w_d - r->prev_d_input) / dt;
    }

    u_int   = r->ki * error;
    pre_sat = p + r->i_state + d;
    dz      = ref_deadzone(pre_sat, r->out_min, r->out_max);

    if (r->clamp_i) {
        if ((dz > 0.0 && u_int > 0.0) || (dz < 0.0 && u_int < 0.0)) u_int = 0.0;
    } else if (r->backcalc_i) {
        u_int += r->kb * (-dz);
    }

    if (r->trapezoid) {
        i_out  = ref_clamp(r->i_state + 0.5 * dt * u_int, r->i_min, r->i_max);
        i_next = ref_clamp(r->i_state + dt * u_int,       r->i_min, r->i_max);
    } else {
        i_out  = ref_clamp(r->i_state + dt * u_int,       r->i_min, r->i_max);
        i_next = i_out;
    }
    r->i_state = i_next;
    r->prev_d_input = w_d;

    return ref_clamp(p + i_out + d, r->out_min, r->out_max);
}

/* ------------------------------------------------------------------------- */
/* Tín hiệu thử: bậc thang + dốc + hình sin + nhiễu. Giống hệt tín hiệu mà    */
/* verify_shark_vs_pid2.m đưa vào cả hai khối trong Simulink.                 */
/* ------------------------------------------------------------------------- */
static void test_signal(int k, double dt, double *sp, double *meas)
{
    double t = (double)k * dt;
    double r;

    if      (t < 0.20) r = 0.0;
    else if (t < 1.00) r = 10.0;
    else if (t < 2.00) r = 10.0 + 8.0 * (t - 1.0);      /* dốc */
    else               r = -6.0;                        /* bậc âm */

    *sp = r;
    /* Giá trị đo: bám trễ theo setpoint + sin + nhiễu -> ép khâu D làm việc. */
    *meas = 0.85 * r * (1.0 - exp(-t * 2.0))
          + 1.5 * sin(t * 9.0)
          + 0.30 * frand_pm1();
}

typedef struct {
    const char *name;
    float kp, ki, kd, b, c, n;
    float out_min, out_max, i_min, i_max, kb;
    uint32_t flags;
    float ts;
} ref_case_t;

#define TRAP  ((uint32_t)SHARK_PID_F_TRAPEZOID_I)
#define CLMP  ((uint32_t)SHARK_PID_F_CLAMP_I)
#define BCLC  ((uint32_t)SHARK_PID_F_BACKCALC_I)
#define WIDE  1.0e6f

static void test_block_reference(void)
{
    static const ref_case_t cases[] = {
        /* name                        kp   ki    kd    b    c     n     omin   omax   imin   imax   kb    flags        ts     */
        { "P thuan",                  4.0f, 0.0f, 0.0f, 1.0f, 0.0f, 100.0f, -WIDE,  WIDE, -WIDE,  WIDE, 0.0f, TRAP|CLMP, 1e-3f },
        { "PI hinh thang",            4.0f, 1.5f, 0.0f, 1.0f, 0.0f, 100.0f, -WIDE,  WIDE, -WIDE,  WIDE, 0.0f, TRAP|CLMP, 1e-3f },
        { "PI backward euler",        4.0f, 1.5f, 0.0f, 1.0f, 0.0f, 100.0f, -WIDE,  WIDE, -WIDE,  WIDE, 0.0f, CLMP,      1e-3f },
        { "PID loc N=250",            4.0f, 1.5f, 0.1f, 1.0f, 0.0f, 250.0f, -WIDE,  WIDE, -WIDE,  WIDE, 0.0f, TRAP|CLMP, 1e-3f },
        { "PID loc N=10",             4.0f, 1.5f, 0.1f, 1.0f, 0.0f,  10.0f, -WIDE,  WIDE, -WIDE,  WIDE, 0.0f, TRAP|CLMP, 1e-3f },
        { "PID khong loc (n<=0)",     4.0f, 1.5f, 0.1f, 1.0f, 0.0f,   0.0f, -WIDE,  WIDE, -WIDE,  WIDE, 0.0f, TRAP|CLMP, 1e-3f },
        { "b=0.7 c=1",                4.0f, 1.5f, 0.1f, 0.7f, 1.0f,  50.0f, -WIDE,  WIDE, -WIDE,  WIDE, 0.0f, TRAP|CLMP, 1e-3f },
        { "b=0 (P-on-measurement)",   4.0f, 1.5f, 0.1f, 0.0f, 0.0f,  50.0f, -WIDE,  WIDE, -WIDE,  WIDE, 0.0f, TRAP|CLMP, 1e-3f },
        { "bao hoa + clamping",       4.0f, 1.5f, 0.1f, 1.0f, 0.0f,  50.0f, -10.0f, 10.0f, -WIDE,  WIDE, 0.0f, TRAP|CLMP, 1e-3f },
        { "bao hoa + back-calc kb=2", 4.0f, 1.5f, 0.1f, 1.0f, 0.0f,  50.0f, -10.0f, 10.0f, -WIDE,  WIDE, 2.0f, TRAP|BCLC, 1e-3f },
        { "bao hoa + back-calc kb=8", 4.0f, 1.5f, 0.1f, 1.0f, 0.0f,  50.0f, -10.0f, 10.0f, -WIDE,  WIDE, 8.0f, TRAP|BCLC, 1e-3f },
        { "bao hoa, khong chong wu",  4.0f, 1.5f, 0.1f, 1.0f, 0.0f,  50.0f, -10.0f, 10.0f, -WIDE,  WIDE, 0.0f, TRAP,      1e-3f },
        { "kep I hinh thang",         4.0f, 3.0f, 0.1f, 1.0f, 0.0f,  50.0f, -20.0f, 20.0f,  -5.0f,  5.0f, 0.0f, TRAP|CLMP, 1e-3f },
        { "kep I backward euler",     4.0f, 3.0f, 0.1f, 1.0f, 0.0f,  50.0f, -20.0f, 20.0f,  -5.0f,  5.0f, 0.0f, CLMP,      1e-3f },
        { "can lech (imin=0)",        4.0f, 3.0f, 0.1f, 1.0f, 0.0f,  50.0f,   0.0f, 20.0f,   0.0f, 12.0f, 0.0f, TRAP|CLMP, 1e-3f },
        { "Ts = 20 ms",               4.0f, 1.5f, 0.1f, 1.0f, 0.0f,  50.0f, -WIDE,  WIDE, -WIDE,  WIDE, 0.0f, TRAP|CLMP, 2e-2f },
        { "Ts = 5 ms + bao hoa",      2.0f, 5.0f, 0.05f,0.8f, 0.0f,  30.0f,  -5.0f,  5.0f,  -4.0f,  4.0f, 0.0f, TRAP|CLMP, 5e-3f }
    };
    const double TOL_REL = 5e-5;    /* sàn làm tròn float vs double đo được ~4e-6 */
    size_t ci;
    double worst_all = 0.0;

    section("0. Doi chieu voi phuong trinh sai phan cua khoi PID (2DOF)");

    for (ci = 0; ci < sizeof cases / sizeof cases[0]; ++ci) {
        const ref_case_t *tc = &cases[ci];
        shark_pid_cfg_t cfg;
        shark_pid_t pid;
        ref_pid_t ref;
        double dt = (double)tc->ts;
        double worst = 0.0, scale = 1.0;
        int k;
        int n_steps = (int)(3.0 / dt + 0.5);

        shark_pid_cfg_default(&cfg);
        cfg.kp = tc->kp;  cfg.ki = tc->ki;  cfg.kd = tc->kd;
        cfg.b  = tc->b;   cfg.c  = tc->c;   cfg.n  = tc->n;
        cfg.out_min = tc->out_min;  cfg.out_max = tc->out_max;
        cfg.i_min   = tc->i_min;    cfg.i_max   = tc->i_max;
        cfg.kb      = tc->kb;       cfg.flags   = tc->flags;
        cfg.dt_nominal = tc->ts;
        cfg.dt_max     = 1.0f;

        shark_pid_init(&pid, &cfg);
        ref_init(&ref, &cfg);
        rng_reset();

        for (k = 0; k < n_steps; ++k) {
            double sp, meas, u_ref, u_c;

            test_signal(k, dt, &sp, &meas);

            u_ref = ref_update(&ref, sp, meas, dt);
            u_c   = (double)shark_pid_update(&pid, (float)sp, (float)meas,
                                             (float)dt);

            if (fabs(u_ref) > scale) scale = fabs(u_ref);
            if (fabs(u_c - u_ref) > worst) worst = fabs(u_c - u_ref);
        }

        worst /= scale;
        if (worst > worst_all) worst_all = worst;

        check(worst < TOL_REL, tc->name,
              fmt("sai so tuong doi %.2e (nguong %.0e)", worst, TOL_REL));
    }

    printf("  -> sai so lon nhat tren %d cau hinh: %.2e\n",
           (int)(sizeof cases / sizeof cases[0]), worst_all);
}

/* ========================================================================= */
/* Mô hình đối tượng: lò sấy bậc nhất                                        */
/*   y' = (K*u - (y - y0)) / tau                                             */
/* ========================================================================= */

#define PLANT_K     1.2
#define PLANT_TAU   8.0
#define PLANT_Y0    20.0
#define MICRO_STEP  1e-4

typedef struct {
    double y_final;
    double peak;
    double t90;         /* < 0 nếu không bao giờ đạt 90% */
} sim_result_t;

static sim_result_t sim_shark(shark_pid_t *pid, double ctrl_dt, double t_end,
                              double setpoint)
{
    sim_result_t r;
    int n_inner = (int)(ctrl_dt / MICRO_STEP + 0.5);
    int n_ctrl  = (int)(t_end / ctrl_dt + 0.5);
    double y = PLANT_Y0;
    double target90 = PLANT_Y0 + (setpoint - PLANT_Y0) * 0.9;
    int k, j;

    r.peak = PLANT_Y0;
    r.t90 = -1.0;

    for (k = 0; k < n_ctrl; ++k) {
        double u = (double)shark_pid_update(pid, (float)setpoint, (float)y,
                                            (float)ctrl_dt);
        for (j = 0; j < n_inner; ++j) {
            y += ((PLANT_K * u - (y - PLANT_Y0)) / PLANT_TAU) * MICRO_STEP;
            if (y > r.peak) r.peak = y;
            if (r.t90 < 0.0 && y >= target90) r.t90 = (double)k * ctrl_dt;
        }
    }
    r.y_final = y;
    return r;
}

/* ------------------------------------------------------------------------- */
/* PID kiểu cũ: KHÔNG nhân dt, hệ số lọc alpha cố định — dùng để đối chứng   */
/* về tính độc lập tần số lấy mẫu.                                           */
/* ------------------------------------------------------------------------- */
typedef struct {
    double kp, ki, kd, out_max, i_max, alpha_d;
    double iout, last_meas, d_last;
} legacy_pid_t;

static double legacy_update(legacy_pid_t *p, double sp, double meas)
{
    double err = sp - meas;
    double pout = p->kp * err;
    double iterm = p->ki * err;                       /* thiếu * dt */
    double dout = p->kd * (p->last_meas - meas);      /* thiếu / dt */
    double out;

    dout = dout * p->alpha_d + p->d_last * (1.0 - p->alpha_d);

    if (fabs(pout + p->iout + dout) > p->out_max && err * p->iout > 0.0) {
        iterm = 0.0;
    }
    p->iout += iterm;
    if (p->iout >  p->i_max) p->iout =  p->i_max;
    if (p->iout < -p->i_max) p->iout = -p->i_max;

    out = pout + p->iout + dout;
    if (out >  p->out_max) out =  p->out_max;
    if (out < 0.0)         out = 0.0;

    p->last_meas = meas;
    p->d_last = dout;
    return out;
}

static sim_result_t sim_legacy(legacy_pid_t *p, double ctrl_dt, double t_end,
                               double setpoint)
{
    sim_result_t r;
    int n_inner = (int)(ctrl_dt / MICRO_STEP + 0.5);
    int n_ctrl  = (int)(t_end / ctrl_dt + 0.5);
    double y = PLANT_Y0;
    int k, j;

    r.peak = PLANT_Y0;
    r.t90 = -1.0;

    for (k = 0; k < n_ctrl; ++k) {
        double u = legacy_update(p, setpoint, y);
        for (j = 0; j < n_inner; ++j) {
            y += ((PLANT_K * u - (y - PLANT_Y0)) / PLANT_TAU) * MICRO_STEP;
            if (y > r.peak) r.peak = y;
        }
    }
    r.y_final = y;
    return r;
}

/* Cấu hình lò sấy dùng chung. */
static void heater_cfg(shark_pid_cfg_t *c, float kp, float ki, float kd)
{
    shark_pid_cfg_default(c);
    c->kp = kp;  c->ki = ki;  c->kd = kd;
    c->out_min = 0.0f;    c->out_max = 100.0f;
    c->i_min   = 0.0f;    c->i_max   = 100.0f;
    c->n = 0.0f;                        /* không lọc D cho các bài đối chứng */
    c->dt_max = 1.0f;
}

/* ========================================================================= */
/* 1. Độc lập tần số lấy mẫu                                                 */
/* ========================================================================= */
static void test_sample_rate_independence(void)
{
    shark_pid_cfg_t c;
    shark_pid_t pa, pb;
    legacy_pid_t la, lb;
    sim_result_t ra, rb, sa, sb;
    double d_shark, d_legacy;

    section("1. Doi tan so vong lap khong phai do lai he so");

    heater_cfg(&c, 2.0f, 0.30f, 1.0f);
    c.dt_nominal = 0.001f;
    shark_pid_init(&pa, &c);
    shark_pid_init(&pb, &c);

    ra = sim_shark(&pa, 0.001, 120.0, 60.0);    /* 1 kHz  */
    rb = sim_shark(&pb, 0.020, 120.0, 60.0);    /* 50 Hz  */

    d_shark = fabs(ra.peak - rb.peak);
    check(d_shark < 0.5, "dinh lech duoi 0.5 do khi doi 20x tan so",
          fmt("1 kHz: %.2f   50 Hz: %.2f", ra.peak, rb.peak));
    check(fabs(ra.y_final - rb.y_final) < 0.1, "xac lap lech duoi 0.1 do",
          fmt("1 kHz: %.3f   50 Hz: %.3f", ra.y_final, rb.y_final));

    memset(&la, 0, sizeof la);
    la.kp = 2.0; la.ki = 0.30; la.kd = 1.0;
    la.out_max = 100.0; la.i_max = 100.0; la.alpha_d = 0.25;
    lb = la;

    sa = sim_legacy(&la, 0.001, 120.0, 60.0);
    sb = sim_legacy(&lb, 0.020, 120.0, 60.0);
    d_legacy = fabs(sa.peak - sb.peak);

    check(d_legacy > 10.0 * d_shark,
          "ban khong nhan dt lech gap hon 10 lan",
          fmt("legacy lech %.2f   shark lech %.2f", d_legacy, d_shark));
}

/* ========================================================================= */
/* 2. Chống windup                                                           */
/* ========================================================================= */
static void test_antiwindup(void)
{
    shark_pid_cfg_t c;
    shark_pid_t p;
    sim_result_t off, on, bc;
    double os_off, os_on, os_bc;
    float kb_list[4];
    double os_prev;
    int monotone = 1, i;

    section("2. Chong windup");

    /* Ki lớn + cơ cấu yếu -> chắc chắn bão hoà lâu khi khởi động nguội.
       Trần tích phân phải nới rộng, nếu không chính i_max đã chặn windup và
       bài test không còn đo được tác dụng của mạch chống windup. */
    heater_cfg(&c, 2.0f, 1.20f, 0.0f);
    c.out_max = 40.0f;              /* cơ cấu yếu: u_xac_lap = 33.3 -> ghim biên rất lâu */
    c.i_min = 0.0f;  c.i_max = 1.0e6f;
    c.dt_nominal = 0.01f;

    c.flags = (uint32_t)SHARK_PID_F_TRAPEZOID_I;             /* không chống */
    shark_pid_init(&p, &c);
    off = sim_shark(&p, 0.01, 300.0, 60.0);
    os_off = off.peak - 60.0;

    c.flags = TRAP | CLMP;                                    /* clamping */
    shark_pid_init(&p, &c);
    on = sim_shark(&p, 0.01, 300.0, 60.0);
    os_on = on.peak - 60.0;

    /* Chốt lại rằng kịch bản THẬT SỰ gây windup — nếu ngõ ra không bão hoà
       lâu thì cả bài test này vô nghĩa, và nó phải hỏng chứ không im lặng. */
    check(os_off > 3.0, "kich ban that su gay windup khi khong chong",
          fmt("vot lo khi tha noi: %.2f do", os_off, 0.0));

    check(os_on < os_off - 3.0, "clamping giam vot lo it nhat 3 do",
          fmt("khong chong: %.2f do   clamping: %.2f do", os_off, os_on));
    check(fabs(on.y_final - 60.0) < 0.5, "van xac lap dung dich",
          fmt("y = %.3f", on.y_final, 0.0));

    /* back-calculation: kb càng lớn kéo về càng mạnh -> vọt lố giảm dần. */
    kb_list[0] = 0.5f;  kb_list[1] = 2.0f;
    kb_list[2] = 5.0f;  kb_list[3] = 12.0f;
    os_prev = 1e9;
    os_bc = 0.0;
    for (i = 0; i < 4; ++i) {
        c.flags = TRAP | BCLC;
        c.kb = kb_list[i];
        shark_pid_init(&p, &c);
        bc = sim_shark(&p, 0.01, 300.0, 60.0);
        os_bc = bc.peak - 60.0;
        if (os_bc > os_prev + 1e-3) monotone = 0;
        os_prev = os_bc;
    }
    check(monotone, "kb cang lon vot lo cang giam (don dieu)", NULL);
    check(os_bc < os_off - 3.0, "back-calculation voi kb lon giam vot lo",
          fmt("khong chong: %.2f do   kb=12: %.2f do", os_off, os_bc));
}

/* ========================================================================= */
/* 3. Kẹp khâu I bằng i_min / i_max                                          */
/* ========================================================================= */
static void test_integrator_limits(void)
{
    shark_pid_cfg_t c;
    shark_pid_t p;
    int k, within_out = 1, within_state = 1;
    const float I_MAX = 25.0f;

    section("3. Kep khau I bang i_min / i_max");

    /* --- 3a. Có bão hoà + clamping ------------------------------------- */
    heater_cfg(&c, 2.0f, 4.0f, 0.0f);
    c.i_min = 0.0f;  c.i_max = I_MAX;
    c.dt_nominal = 0.01f;
    shark_pid_init(&p, &c);

    /* Giữ sai số dương thật lâu: bộ tích phân buộc phải chạy tới hạn. */
    for (k = 0; k < 3000; ++k) {
        (void)shark_pid_update(&p, 60.0f, 20.0f, 0.01f);
        if (p.i_term  < -1e-4f || p.i_term  > I_MAX + 1e-4f) within_out = 0;
        if (p.i_state < -1e-4f || p.i_state > I_MAX + 1e-4f) within_state = 0;
    }

    check(within_out, "NGO RA khau I khong bao gio vuot [i_min, i_max]", NULL);
    check(within_state, "TRANG THAI khau I khong bao gio vuot [i_min, i_max]",
          NULL);

    /* P = kp*(60-20) = 80 và out_max = 100, nên preSat vượt biên ngay khi
       i_state ~ 20 -> clamping ngắt tích phân TRƯỚC khi chạm trần i_max. */
    check(p.i_term < I_MAX - 1.0f,
          "clamping ngat tich phan truoc ca tran i_max",
          fmt("i_term dung o %.3f   (i_max = %.1f)",
              (double)p.i_term, (double)I_MAX));

    /* --- 3b. Không bão hoà: chỉ còn i_max chặn -> phải chạm đúng trần --- */
    heater_cfg(&c, 2.0f, 4.0f, 0.0f);
    c.out_min = -1.0e6f;  c.out_max = 1.0e6f;
    c.i_min   =  0.0f;    c.i_max   = I_MAX;
    c.dt_nominal = 0.01f;
    shark_pid_init(&p, &c);
    for (k = 0; k < 3000; ++k) {
        (void)shark_pid_update(&p, 60.0f, 20.0f, 0.01f);
    }
    check(fabsf(p.i_term - I_MAX) < 1e-3f,
          "cham dung tran i_max khi ngo ra khong bao hoa",
          fmt("i_term = %.6f (i_max = %.1f)",
              (double)p.i_term, (double)I_MAX));

    /* --- 3c. Sai số đổi dấu -> rời trần đi xuống ----------------------- */
    for (k = 0; k < 200; ++k) {
        (void)shark_pid_update(&p, 20.0f, 60.0f, 0.01f);
    }
    check(p.i_term < I_MAX - 1.0f, "sai so doi dau -> khau I roi tran duoc",
          fmt("i_term = %.3f", (double)p.i_term, 0.0));
}

/* ========================================================================= */
/* 4. Derivative kick và setpoint weight c                                   */
/* ========================================================================= */
static void test_derivative_kick(void)
{
    shark_pid_cfg_t c;
    shark_pid_t p;
    float d_c0, d_c1;
    int k;

    section("4. Derivative kick (setpoint weight c)");

    shark_pid_cfg_default(&c);
    c.kp = 2.0f;  c.ki = 0.0f;  c.kd = 5.0f;
    c.n = 0.0f;                                 /* không lọc: kick lộ rõ nhất */
    c.out_min = -1e6f;  c.out_max = 1e6f;
    c.i_min = -1e6f;    c.i_max = 1e6f;
    c.dt_nominal = 0.01f;

    /* c = 0 (mặc định): khâu D chỉ nhìn giá trị đo. */
    c.c = 0.0f;
    shark_pid_init(&p, &c);
    for (k = 0; k < 20; ++k) (void)shark_pid_update(&p, 20.0f, 20.0f, 0.01f);
    (void)shark_pid_update(&p, 60.0f, 20.0f, 0.01f);    /* setpoint nhảy 40 */
    d_c0 = p.d_term;

    check(fabsf(d_c0) < 1e-6f, "c=0 triet tieu hoan toan derivative kick",
          fmt("d_term = %.9f", (double)d_c0, 0.0));

    /* c = 1: khâu D nhìn sai số -> tái hiện được cú kick để đối chứng. */
    c.c = 1.0f;
    shark_pid_init(&p, &c);
    for (k = 0; k < 20; ++k) (void)shark_pid_update(&p, 20.0f, 20.0f, 0.01f);
    (void)shark_pid_update(&p, 60.0f, 20.0f, 0.01f);
    d_c1 = p.d_term;

    /* kd * (delta w_d) / dt = 5 * 40 / 0.01 = 20000 */
    check(fabsf(d_c1 - 20000.0f) < 1.0f, "c=1 cho dung cu kick D*dr/dt",
          fmt("d_term = %.1f (ky vong 20000)", (double)d_c1, 0.0));
}

/* ========================================================================= */
/* 5. Bộ lọc khâu D khai bằng N                                              */
/* ========================================================================= */
static void test_derivative_filter(void)
{
    shark_pid_cfg_t c;
    shark_pid_t p;
    double energy_soft = 0.0, energy_hard = 0.0;
    double d_at_tau_1ms, d_at_tau_5ms, expect;
    int k, n;

    section("5. Bo loc khau D (Filter coefficient N)");

    /* --- 5a. N nhỏ lọc nhiễu mạnh hơn N lớn --------------------------- */
    shark_pid_cfg_default(&c);
    c.kp = 0.0f;  c.ki = 0.0f;  c.kd = 1.0f;  c.c = 0.0f;
    c.out_min = -1e6f;  c.out_max = 1e6f;
    c.i_min = -1e6f;    c.i_max = 1e6f;
    c.dt_nominal = 0.001f;

    c.n = 300.0f;                       /* lọc nhẹ */
    shark_pid_init(&p, &c);
    rng_reset();
    for (k = 0; k < 2000; ++k) {
        double y = 10.0 + 0.5 * frand_pm1();
        (void)shark_pid_update(&p, 10.0f, (float)y, 0.001f);
        if (k > 200) energy_hard += (double)p.d_term * (double)p.d_term;
    }

    c.n = 5.0f;                         /* lọc mạnh */
    shark_pid_init(&p, &c);
    rng_reset();
    for (k = 0; k < 2000; ++k) {
        double y = 10.0 + 0.5 * frand_pm1();
        (void)shark_pid_update(&p, 10.0f, (float)y, 0.001f);
        if (k > 200) energy_soft += (double)p.d_term * (double)p.d_term;
    }

    check(energy_soft * 10.0 < energy_hard,
          "N nho loc nhieu manh hon N lon it nhat 10 lan",
          fmt("nang luong D: N=5 -> %.3g   N=300 -> %.3g",
              energy_soft, energy_hard));

    /* --- 5b. Tần số cắt không trôi khi đổi tần số vòng lặp ------------- */
    /* Đáp ứng bước của nhánh D là N*Kd*A*exp(-N*t). Sau đúng t = 1/N nó
       phải còn ~1/e lần đỉnh, BẤT KỂ dt — vì hệ số suy lại từ dt mỗi chu kỳ. */
    shark_pid_cfg_default(&c);
    c.kp = 0.0f;  c.ki = 0.0f;  c.kd = 1.0f;  c.c = 1.0f;
    c.n = 20.0f;                        /* hằng số thời gian 50 ms */
    c.out_min = -1e6f;  c.out_max = 1e6f;
    c.i_min = -1e6f;    c.i_max = 1e6f;
    c.dt_max = 1.0f;

    c.dt_nominal = 0.001f;
    shark_pid_init(&p, &c);
    (void)shark_pid_update(&p, 0.0f, 0.0f, 0.001f);         /* lấy mốc */
    n = (int)(1.0 / (20.0 * 0.001) + 0.5);                  /* t = 1/N */
    for (k = 0; k < n; ++k) (void)shark_pid_update(&p, 1.0f, 0.0f, 0.001f);
    d_at_tau_1ms = (double)p.d_term;

    c.dt_nominal = 0.005f;
    shark_pid_init(&p, &c);
    (void)shark_pid_update(&p, 0.0f, 0.0f, 0.005f);
    n = (int)(1.0 / (20.0 * 0.005) + 0.5);
    for (k = 0; k < n; ++k) (void)shark_pid_update(&p, 1.0f, 0.0f, 0.005f);
    d_at_tau_5ms = (double)p.d_term;

    expect = 20.0 / exp(1.0);           /* N*Kd*A/e */
    check(fabs(d_at_tau_1ms - d_at_tau_5ms) < 0.10 * expect,
          "doi dt 5 lan, tan so cat khong troi",
          fmt("dt=1ms -> %.4f   dt=5ms -> %.4f", d_at_tau_1ms, d_at_tau_5ms));
    check(fabs(d_at_tau_1ms - expect) < 0.10 * expect,
          "bam sat duong cong lien tuc N*Kd*A*exp(-N*t)",
          fmt("do duoc %.4f   ly thuyet %.4f", d_at_tau_1ms, expect));

    /* --- 5c. n <= 0 là hiệu lùi thuần, không lọc ---------------------- */
    shark_pid_cfg_default(&c);
    c.kp = 0.0f;  c.ki = 0.0f;  c.kd = 2.0f;  c.c = 1.0f;  c.n = -1.0f;
    c.out_min = -1e6f;  c.out_max = 1e6f;
    c.i_min = -1e6f;    c.i_max = 1e6f;
    shark_pid_init(&p, &c);
    (void)shark_pid_update(&p, 0.0f, 0.0f, 0.01f);
    (void)shark_pid_update(&p, 3.0f, 0.0f, 0.01f);
    /* kd * (3 - 0) / 0.01 = 600 */
    check(fabsf(p.d_term - 600.0f) < 1e-2f, "n <= 0 -> D = Kd*(w - w_truoc)/dt",
          fmt("d_term = %.3f (ky vong 600)", (double)p.d_term, 0.0));
}

/* ========================================================================= */
/* 6. Chặn NaN/Inf và bảo vệ dt                                              */
/* ========================================================================= */
static void test_bad_input_and_dt(void)
{
    shark_pid_cfg_t c;
    shark_pid_t p;
    float good, after;
    int k;

    section("6. Chan NaN/Inf va bao ve dt");

    heater_cfg(&c, 2.0f, 0.5f, 0.5f);
    c.dt_nominal = 0.01f;
    c.dt_max = 0.5f;
    shark_pid_init(&p, &c);

    for (k = 0; k < 10; ++k) good = shark_pid_update(&p, 60.0f, 30.0f, 0.01f);

    (void)shark_pid_update(&p, (float)NAN, 30.0f, 0.01f);
    check(p.output == good, "NaN -> giu nguyen lenh cu",
          fmt("truoc %.4f   sau %.4f", (double)good, (double)p.output));
    check((p.status & (uint32_t)SHARK_PID_BAD_INPUT) != 0u,
          "NaN -> bat co BAD_INPUT", NULL);

    (void)shark_pid_update(&p, 60.0f, (float)INFINITY, 0.01f);
    check(isfinite((double)p.i_term), "Inf khong lot vao bo tich phan",
          fmt("i_term = %.4f", (double)p.i_term, 0.0));

    after = shark_pid_update(&p, 60.0f, 30.0f, 0.01f);
    check(isfinite((double)after) && after > 0.0f,
          "chu ky sach ngay sau do van chay binh thuong",
          fmt("u = %.4f", (double)after, 0.0));

    (void)shark_pid_update(&p, 60.0f, 30.0f, 0.0f);
    check((p.status & (uint32_t)SHARK_PID_BAD_DT) != 0u,
          "dt = 0 -> bat co BAD_DT", NULL);
    check(p.dt_used == c.dt_nominal, "dt = 0 -> thay bang dt_nominal",
          fmt("dt_used = %.6f", (double)p.dt_used, 0.0));

    (void)shark_pid_update(&p, 60.0f, 30.0f, -0.5f);
    check((p.status & (uint32_t)SHARK_PID_BAD_DT) != 0u,
          "dt am -> bat co BAD_DT", NULL);

    (void)shark_pid_update(&p, 60.0f, 30.0f, 5.0f);
    check((p.status & (uint32_t)SHARK_PID_BAD_DT) != 0u,
          "dt > dt_max -> bat co BAD_DT", NULL);

    (void)shark_pid_update(&p, 60.0f, 30.0f, (float)NAN);
    check((p.status & (uint32_t)SHARK_PID_BAD_DT) != 0u,
          "dt = NaN -> bat co BAD_DT", NULL);
    check(isfinite((double)p.output),
          "moi truong hop dt hong van cho ngo ra huu han", NULL);

    /* Cờ tính lại mỗi chu kỳ, không chốt lại. */
    (void)shark_pid_update(&p, 60.0f, 30.0f, 0.01f);
    check((p.status & (uint32_t)SHARK_PID_BAD_DT) == 0u,
          "co BAD_DT tu xoa o chu ky sach", NULL);
}

/* ========================================================================= */
/* 7. Đổi hệ số lúc đang chạy không làm giật ngõ ra                          */
/* ========================================================================= */
static void test_bumpless_gain_change(void)
{
    shark_pid_cfg_t c;
    shark_pid_t p;
    float before, after, i_before;
    double jump, one_step, naive_jump;
    int k;

    section("7. Doi he so luc dang chay khong giat");

    heater_cfg(&c, 2.0f, 0.40f, 0.0f);
    c.dt_nominal = 0.01f;
    shark_pid_init(&p, &c);

    /* Chạy tới gần xác lập để bộ tích phân tích được lịch sử dài. */
    for (k = 0; k < 6000; ++k) {
        double y = 60.0 - 40.0 * exp(-(double)k * 0.01 / 8.0);
        before = shark_pid_update(&p, 60.0f, (float)y, 0.01f);
    }
    i_before = p.i_term;

    shark_pid_set_gains(&p, 2.0f, 1.60f, 0.0f);     /* Ki gấp 4 lần */
    after = shark_pid_update(&p, 60.0f, 60.0f, 0.01f);

    jump = fabs((double)after - (double)before);
    one_step = 1.60 * 0.01 * 40.0;      /* trần: một bước tích phân hệ số mới */
    naive_jump = fabs((double)i_before * 4.0 - (double)i_before);

    check(jump <= one_step + 1e-3, "nhay khong vuot mot buoc tich phan",
          fmt("nhay %.5f   tran %.5f", jump, one_step));
    check(jump * 20.0 < naive_jump,
          "nho hon han cach cai dat ngay tho i = Ki*tong(e)",
          fmt("shark %.5f   ngay tho %.5f", jump, naive_jump));

    /* set_gains(ki = 0) phải xoá sạch bộ tích phân. */
    shark_pid_set_gains(&p, 2.0f, 0.0f, 0.0f);
    check(p.i_term == 0.0f && p.i_state == 0.0f,
          "dat ki = 0 xoa sach bo tich phan", NULL);
}

/* ========================================================================= */
/* 8. Chuyển tay -> tự động không giật (preload)                             */
/* ========================================================================= */
static void test_preload(void)
{
    shark_pid_cfg_t c;
    shark_pid_t p;
    float u;

    section("8. Chuyen tay -> tu dong khong giat");

    heater_cfg(&c, 2.0f, 0.5f, 1.0f);
    c.dt_nominal = 0.01f;
    shark_pid_init(&p, &c);

    shark_pid_preload(&p, 40.0f);
    check(fabsf(p.i_state - 40.0f) < 1e-6f, "preload nap dung bo tich phan",
          fmt("i_state = %.6f", (double)p.i_state, 0.0));

    /* Ở xác lập (r = y) khâu P và D đều bằng 0 -> lệnh đầu tiên bám preload. */
    u = shark_pid_update(&p, 45.0f, 45.0f, 0.01f);
    check(fabsf(u - 40.0f) < 0.5f, "lenh dau tien bam sat gia tri nap truoc",
          fmt("u = %.4f (nap 40)", (double)u, 0.0));

    /* preload bị kẹp vào dải ngõ ra. */
    shark_pid_preload(&p, 500.0f);
    check(p.i_state <= 100.0f, "preload bi kep vao [out_min, out_max]",
          fmt("i_state = %.3f", (double)p.i_state, 0.0));

    /* Giá trị vô nghĩa thì bỏ qua, không đầu độc trạng thái. */
    shark_pid_preload(&p, (float)NAN);
    check(isfinite((double)p.i_state), "preload NaN bi bo qua", NULL);
}

/* ========================================================================= */
/* 9. Bất biến API                                                           */
/* ========================================================================= */
static void test_api_invariants(void)
{
    shark_pid_cfg_t c;
    shark_pid_t p;
    int k, within = 1;

    section("9. Bat bien API");

    heater_cfg(&c, 5.0f, 2.0f, 1.0f);
    c.out_min = -30.0f;  c.out_max = 70.0f;
    c.dt_nominal = 0.01f;
    shark_pid_init(&p, &c);

    rng_reset();
    for (k = 0; k < 5000; ++k) {
        double sp = 40.0 * frand_pm1();
        double y  = 40.0 * frand_pm1();
        float u = shark_pid_update(&p, (float)sp, (float)y, 0.01f);
        if (u < -30.0f - 1e-4f || u > 70.0f + 1e-4f) within = 0;
        if (!isfinite((double)u)) within = 0;
    }
    check(within, "ngo ra luon nam trong [out_min, out_max] va luon huu han",
          NULL);

    shark_pid_reset(&p);
    check(p.i_term == 0.0f && p.i_state == 0.0f && p.output == 0.0f
          && p.d_state == 0.0f && p.status == (uint32_t)SHARK_PID_OK,
          "reset xoa sach trang thai", NULL);
    check(p.cfg.kp == 5.0f && p.cfg.out_max == 70.0f,
          "reset giu nguyen cau hinh", NULL);

    shark_pid_set_output_limits(&p, -5.0f, 10.0f);
    check(p.cfg.out_min == -5.0f && p.cfg.out_max == 10.0f,
          "set_output_limits doi duoc dai ngo ra", NULL);

    /* Truyền ngược thứ tự thì tự hoán đổi, không sinh dải rỗng. */
    shark_pid_set_output_limits(&p, 20.0f, 8.0f);
    check(p.cfg.out_min == 8.0f && p.cfg.out_max == 20.0f,
          "set_output_limits tu hoan doi khi truyen nguoc", NULL);

    /* An toàn với con trỏ NULL. */
    shark_pid_init(NULL, &c);
    shark_pid_reset(NULL);
    shark_pid_cfg_default(NULL);
    shark_pid_set_gains(NULL, 1.0f, 1.0f, 1.0f);
    shark_pid_set_output_limits(NULL, 0.0f, 1.0f);
    shark_pid_preload(NULL, 1.0f);
    check(shark_pid_update(NULL, 1.0f, 1.0f, 0.01f) == 0.0f,
          "moi ham chiu duoc con tro NULL", NULL);

    /* Cấu hình mặc định đúng như tài liệu README mục 6.5. */
    shark_pid_cfg_default(&c);
    check(c.b == 1.0f && c.c == 0.0f && c.n == 100.0f
          && c.dt_max == 0.5f && c.dt_nominal == 0.001f
          && c.flags == (TRAP | CLMP),
          "cfg_default dung nhu tai lieu", NULL);

    /* init(NULL cfg) = dùng mặc định. */
    shark_pid_init(&p, NULL);
    check(p.cfg.n == 100.0f && p.cfg.b == 1.0f,
          "init voi cfg = NULL dung cau hinh mac dinh", NULL);
}

/* ========================================================================= */
/* 10. Bẫy b < 1: khâu I phải bù Kp*r*(1-b)                                  */
/* ========================================================================= */
static void test_setpoint_weight_trap(void)
{
    shark_pid_cfg_t c;
    shark_pid_t p1, p2;
    sim_result_t r1, r2;
    float i_b1, i_b07;
    double predicted, diff;
    const double R = 60.0;
    const float KP = 2.0f;
    const float B  = 0.7f;

    section("10. Bay b < 1 (khau I phai bu Kp*r*(1-b))");

    heater_cfg(&c, KP, 0.30f, 0.0f);
    c.dt_nominal = 0.01f;

    c.b = 1.0f;
    shark_pid_init(&p1, &c);
    r1 = sim_shark(&p1, 0.01, 600.0, R);
    i_b1 = p1.i_term;

    c.b = B;
    shark_pid_init(&p2, &c);
    r2 = sim_shark(&p2, 0.01, 600.0, R);
    i_b07 = p2.i_term;

    check(fabs(r1.y_final - R) < 0.1 && fabs(r2.y_final - R) < 0.1,
          "ca hai deu ve dung dich khi i_max du rong",
          fmt("b=1: %.3f   b=0.7: %.3f", r1.y_final, r2.y_final));

    predicted = (double)KP * R * (1.0 - (double)B);     /* 2 * 60 * 0.3 = 36 */
    diff = (double)i_b07 - (double)i_b1;
    check(fabs(diff - predicted) < 0.5,
          "khau I bu dung luong Kp*r*(1-b)",
          fmt("do duoc %.3f   du doan %.3f", diff, predicted));

    check(fabs((double)p2.p_term + predicted) < 0.5,
          "khau P xuat ra dung -Kp*r*(1-b) o xac lap",
          fmt("p_term = %.3f   du doan %.3f", (double)p2.p_term, -predicted));

    /* i_max quá nhỏ -> sai số xác lập vĩnh viễn, đúng như README cảnh báo. */
    c.i_max = 20.0f;                    /* < 36 + tai tinh */
    shark_pid_init(&p2, &c);
    r2 = sim_shark(&p2, 0.01, 600.0, R);
    check(R - r2.y_final > 1.0,
          "i_max nho hon Kp*r*(1-b) -> ket sai so xac lap",
          fmt("dich %.1f   dung lai o %.3f", R, r2.y_final));
}

/* ========================================================================= */
int main(void)
{
    printf("shark_pid %s — bo test chay tren may tinh\n", SHARK_PID_VERSION_STR);

    test_block_reference();
    test_sample_rate_independence();
    test_antiwindup();
    test_integrator_limits();
    test_derivative_kick();
    test_derivative_filter();
    test_bad_input_and_dt();
    test_bumpless_gain_change();
    test_preload();
    test_api_invariants();
    test_setpoint_weight_trap();

    printf("\n=========================================\n");
    printf("  dat: %d    hong: %d\n", g_pass, g_fail);
    printf("=========================================\n");

    return g_fail == 0 ? 0 : 1;
}
