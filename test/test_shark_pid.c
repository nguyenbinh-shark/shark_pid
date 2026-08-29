/**
 * @file    test_shark_pid.c
 * @brief   Bộ test chạy trên máy tính, không cần vi điều khiển và không cần
 *          thư viện test bên ngoài. Trả về 0 nếu tất cả đạt.
 *
 * Biên dịch và chạy:
 *   cc -std=c99 -O2 -Wall -Wextra -I../src ../src/shark_pid.c test_shark_pid.c -lm -o test
 *   ./test
 *
 * Hoặc:  make test
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
    static char buf[160];
    snprintf(buf, sizeof buf, f, a, b);
    return buf;
}

/* ========================================================================= */
/* Mô hình đối tượng: lò sấy bậc nhất                                        */
/*   y' = (K*u - (y - y0)) / tau                                             */
/* Bước thời gian bằng số nguyên để kết quả tái lập được chính xác.          */
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
                              double setpoint, double umin, double umax)
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
        if (u < umin) u = umin;
        if (u > umax) u = umax;

        for (j = 0; j < n_inner; ++j) {
            y += ((PLANT_K * u - (y - PLANT_Y0)) / PLANT_TAU) * MICRO_STEP;
            if (y > r.peak) r.peak = y;
            if (r.t90 < 0.0 && y >= target90) r.t90 = k * ctrl_dt;
        }
    }
    r.y_final = y;
    return r;
}

/* ------------------------------------------------------------------------- */
/* Bản PID kiểu cũ: KHÔNG nhân dt, hệ số lọc cố định.                        */
/* Dùng để chứng minh khác biệt về tính độc lập tần số lấy mẫu.              */
/* ------------------------------------------------------------------------- */
typedef struct {
    double kp, ki, kd, out_max, i_max, alpha_d;
    double iout, last_meas, d_last;
} legacy_pid_t;

static void legacy_init(legacy_pid_t *p, double kp, double ki, double kd,
                        double out_max, double i_max)
{
    memset(p, 0, sizeof *p);
    p->kp = kp; p->ki = ki; p->kd = kd;
    p->out_max = out_max; p->i_max = i_max;
    p->alpha_d = 0.25;
}

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
    if (out < -p->out_max) out = -p->out_max;

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
        if (u < 0.0) u = 0.0;
        if (u > 100.0) u = 100.0;
        for (j = 0; j < n_inner; ++j) {
            y += ((PLANT_K * u - (y - PLANT_Y0)) / PLANT_TAU) * MICRO_STEP;
            if (y > r.peak) r.peak = y;
        }
    }
    r.y_final = y;
    return r;
}

/* Cấu hình gốc dùng chung cho các bài test lò sấy. */
static void heater_cfg(shark_pid_cfg_t *c, float kp, float ki, float kd)
{
    shark_pid_cfg_default(c);
    c->kp = kp; c->ki = ki; c->kd = kd;
    c->out_min = 0.0f;   c->out_max = 100.0f;
    c->i_min   = 0.0f;   c->i_max   = 100.0f;
    c->d_tau = 0.0f;
}

/* ========================================================================= */
/* 1. Độc lập tần số lấy mẫu                                                 */
/* ========================================================================= */
static void test_sample_rate_independence(void)
{
    shark_pid_t a, b;
    shark_pid_cfg_t ca, cb;
    legacy_pid_t la, lb;
    sim_result_t ra, rb, rla, rlb;
    double d_shark, d_legacy;

    section("1. Doc lap tan so lay mau (1 kHz vs 50 Hz, cung he so)");

    heater_cfg(&ca, 3.0f, 0.8f, 1.5f);
    ca.d_tau = 0.05f;  ca.dt_nominal = 0.001f;
    cb = ca;           cb.dt_nominal = 0.020f;

    shark_pid_init(&a, &ca);
    shark_pid_init(&b, &cb);

    ra = sim_shark(&a, 0.001, 60.0, 60.0, 0.0, 100.0);
    rb = sim_shark(&b, 0.020, 60.0, 60.0, 0.0, 100.0);
    d_shark = fabs(ra.peak - rb.peak);

    printf("   shark  1kHz peak=%.4f  50Hz peak=%.4f\n", ra.peak, rb.peak);
    check(d_shark < 0.5, "dinh lech duoi 0.5 do khi doi 20x tan so",
          fmt("lech=%.4f (%.4f)", d_shark, d_shark));
    check(fabs(ra.y_final - rb.y_final) < 0.1, "xac lap lech duoi 0.1 do",
          fmt("%.5f vs %.5f", ra.y_final, rb.y_final));

    legacy_init(&la, 3.0, 0.8, 1.5, 100.0, 100.0);
    legacy_init(&lb, 3.0, 0.8, 1.5, 100.0, 100.0);
    rla = sim_legacy(&la, 0.001, 60.0, 60.0);
    rlb = sim_legacy(&lb, 0.020, 60.0, 60.0);
    d_legacy = fabs(rla.peak - rlb.peak);

    printf("   legacy 1kHz peak=%.4f  50Hz peak=%.4f\n", rla.peak, rlb.peak);
    check(d_legacy > 20.0 * d_shark,
          "ban khong co dt troi gap it nhat 20 lan",
          fmt("legacy=%.4f  shark=%.4f", d_legacy, d_shark));
}

/* ========================================================================= */
/* 2. Chống windup                                                           */
/* ========================================================================= */
static void test_antiwindup(void)
{
    shark_pid_t p;
    shark_pid_cfg_t c;
    sim_result_t on, off, bc;
    double os_on, os_off, os_prev, os_bc = 0.0;
    int monotone = 1;
    const float kts[5] = { 0.1f, 0.33f, 1.0f, 3.0f, 10.0f };
    int i;

    section("2. Chong windup (co cau bao hoa 0-100%, muc tieu 60 do)");

    heater_cfg(&c, 3.0f, 1.2f, 0.0f);
    c.flags = SHARK_PID_F_TRAPEZOID_I | SHARK_PID_F_CLAMP_I;
    shark_pid_init(&p, &c);
    on = sim_shark(&p, 0.01, 120.0, 60.0, 0.0, 100.0);
    os_on = (on.peak - 60.0) / 60.0 * 100.0;

    c.flags = SHARK_PID_F_TRAPEZOID_I;
    shark_pid_init(&p, &c);
    off = sim_shark(&p, 0.01, 120.0, 60.0, 0.0, 100.0);
    os_off = (off.peak - 60.0) / 60.0 * 100.0;

    printf("   kep bat = %.2f%%   kep tat = %.2f%%\n", os_on, os_off);
    check(os_on < os_off - 3.0, "kep co dieu kien giam vot lo it nhat 3 diem",
          fmt("%.2f%% vs %.2f%%", os_on, os_off));
    check(fabs(on.y_final - 60.0) < 0.5, "van xac lap dung dich",
          fmt("y=%.4f (%.1f)", on.y_final, 60.0));

    os_prev = 1e9;
    for (i = 0; i < 5; ++i) {
        c.flags = SHARK_PID_F_TRAPEZOID_I | SHARK_PID_F_BACKCALC_I;
        c.kt = kts[i];
        shark_pid_init(&p, &c);
        bc = sim_shark(&p, 0.01, 120.0, 60.0, 0.0, 100.0);
        os_bc = (bc.peak - 60.0) / 60.0 * 100.0;
        printf("   back-calc kt=%5.2f -> %.2f%%\n", (double)kts[i], os_bc);
        if (os_bc > os_prev + 1e-6) monotone = 0;
        os_prev = os_bc;
    }
    check(monotone, "kt cang lon vot lo cang giam (don dieu)", NULL);
    check(os_bc < os_off - 3.0, "back-calc voi kt hop ly giam vot lo",
          fmt("kt=10 -> %.2f%% vs %.2f%%", os_bc, os_off));
}

/* ========================================================================= */
/* 3. Derivative kick                                                        */
/* ========================================================================= */
static void test_derivative_kick(void)
{
    shark_pid_t p;
    shark_pid_cfg_t c;
    int i;

    section("3. Derivative kick (nhay setpoint 30 -> 60, gia tri do dung yen)");

    shark_pid_cfg_default(&c);
    c.kp = 2.0f;  c.ki = 0.0f;  c.kd = 5.0f;
    c.out_min = -500.0f;  c.out_max = 500.0f;
    c.d_tau = 0.0f;

    c.c = 0.0f;                                  /* D-on-measurement */
    shark_pid_init(&p, &c);
    for (i = 0; i < 50; ++i) shark_pid_update(&p, 30.0f, 30.0f, 0.01f);
    shark_pid_update(&p, 60.0f, 30.0f, 0.01f);
    printf("   c=0 (D theo gia tri do) : D = %+.4f\n", (double)p.d_term);
    check(fabsf(p.d_term) < 1e-6f, "c=0 triet tieu hoan toan derivative kick",
          fmt("D=%.6f (%.1f)", (double)p.d_term, 0.0));

    c.c = 1.0f;                                  /* D-on-error, kieu co dien */
    shark_pid_init(&p, &c);
    for (i = 0; i < 50; ++i) shark_pid_update(&p, 30.0f, 30.0f, 0.01f);
    shark_pid_update(&p, 60.0f, 30.0f, 0.01f);
    printf("   c=1 (D theo sai so)     : D = %+.4f\n", (double)p.d_term);
    check(p.d_term > 100.0f, "c=1 tai hien duoc cu kick de doi chieu",
          fmt("D=%.1f (%.1f)", (double)p.d_term, 100.0));
}

/* ========================================================================= */
/* 4. Vùng chết không sinh xung vi phân giả                                  */
/* ========================================================================= */
static void test_deadband_no_fake_derivative(void)
{
    shark_pid_t p;
    shark_pid_cfg_t c;
    const float KD = 4.0f, DT = 0.01f;
    float d_true, d_bug;
    int i;

    section("4. Vung chet khong sinh xung D gia");

    shark_pid_cfg_default(&c);
    c.kp = 2.0f;  c.ki = 0.0f;  c.kd = KD;  c.c = 0.0f;
    c.deadband = 1.0f;
    c.out_min = -5000.0f;  c.out_max = 5000.0f;
    c.d_tau = 0.0f;
    shark_pid_init(&p, &c);

    /* Gia tri do chi nhich 0.10, nhung sai so vua vuot tu ngoai vao trong
       vung chet (1.05 -> 0.95). D that phai nho. Cach zero hoa sai so roi
       vi phan se lay (0 - 1.05)/dt -> vot len gap ~10 lan. */
    for (i = 0; i < 50; ++i) shark_pid_update(&p, 60.0f, 58.95f, DT);
    shark_pid_update(&p, 60.0f, 59.05f, DT);

    d_true = KD * (-0.10f) / DT;
    d_bug  = KD * (0.0f - 1.05f) / DT;

    printf("   D cua shark_pid = %+.2f   (ky vong %+.2f)\n",
           (double)p.d_term, (double)d_true);
    printf("   D cua ban co loi = %+.2f  -> gap %.1f lan\n",
           (double)d_bug, (double)fabsf(d_bug / d_true));

    check(fabsf(p.d_term - d_true) < 0.01f, "D bam theo chuyen dong that",
          fmt("%.4f vs %.4f", (double)p.d_term, (double)d_true));
    check(fabsf(p.d_term) < fabsf(d_bug) / 5.0f, "D nho hon han ban co loi",
          fmt("%.2f vs %.2f", (double)fabsf(p.d_term), (double)fabsf(d_bug)));
    check(p.p_term == 0.0f, "khau P bi ngat trong vung chet", NULL);
}

/* ========================================================================= */
/* 5. Miễn nhiễm NaN / Inf                                                   */
/* ========================================================================= */
static void test_nan_immunity(void)
{
    shark_pid_t p;
    shark_pid_cfg_t c;
    float good, after;
    int i;

    section("5. Mien nhiem NaN / Inf");

    heater_cfg(&c, 2.0f, 1.0f, 0.5f);
    shark_pid_init(&p, &c);
    for (i = 0; i < 30; ++i) shark_pid_update(&p, 60.0f, 40.0f, 0.01f);
    good = p.output;

    shark_pid_update(&p, 60.0f, (float)NAN, 0.01f);
    check(p.output == good, "NaN -> giu nguyen lenh cu",
          fmt("%.4f vs %.4f", (double)p.output, (double)good));
    check((p.status & SHARK_PID_BAD_INPUT) != 0u, "NaN -> bat co BAD_INPUT", NULL);

    shark_pid_update(&p, 60.0f, (float)INFINITY, 0.01f);
    check(isfinite((double)p.i_term), "Inf khong lot vao bo tich phan",
          fmt("i_term=%.4f (%.1f)", (double)p.i_term, 0.0));

    after = shark_pid_update(&p, 60.0f, 40.0f, 0.01f);
    check(isfinite((double)after) && after > 0.0f,
          "sau su co bo dieu khien chay lai binh thuong",
          fmt("u=%.4f (%.1f)", (double)after, 0.0));

    /* dt hong cung phai bi chan */
    shark_pid_update(&p, 60.0f, 40.0f, 0.0f);
    check((p.status & SHARK_PID_BAD_DT) != 0u, "dt = 0 -> bat co BAD_DT", NULL);
    check(p.dt_used == c.dt_nominal, "dt = 0 -> thay bang dt_nominal",
          fmt("%.6f vs %.6f", (double)p.dt_used, (double)c.dt_nominal));

    shark_pid_update(&p, 60.0f, 40.0f, -1.0f);
    check((p.status & SHARK_PID_BAD_DT) != 0u, "dt am -> bat co BAD_DT", NULL);
    shark_pid_update(&p, 60.0f, 40.0f, 999.0f);
    check((p.status & SHARK_PID_BAD_DT) != 0u, "dt qua lon -> bat co BAD_DT", NULL);
    check(isfinite((double)p.output), "moi truong hop dt hong van cho ngo ra huu han",
          NULL);
}

/* ========================================================================= */
/* 6. Giới hạn dốc ngõ ra                                                    */
/* ========================================================================= */
static void test_slew_limit(void)
{
    shark_pid_t p;
    shark_pid_cfg_t c;
    float prev = 0.0f, max_step = 0.0f;
    const float SLEW = 200.0f, DT = 0.01f;
    int i;

    section("6. Gioi han doc ngo ra");

    shark_pid_cfg_default(&c);
    c.kp = 50.0f;  c.ki = 0.0f;  c.kd = 0.0f;
    c.out_min = -300.0f;  c.out_max = 300.0f;
    c.out_slew = SLEW;
    c.d_tau = 0.0f;
    shark_pid_init(&p, &c);

    for (i = 0; i < 20; ++i) {
        float u = shark_pid_update(&p, 100.0f, 0.0f, DT);
        float step = fabsf(u - prev);
        if (step > max_step) max_step = step;
        prev = u;
    }
    printf("   buoc lon nhat = %.4f   tran ly thuyet = %.4f\n",
           (double)max_step, (double)(SLEW * DT));
    check(max_step <= SLEW * DT + 1e-4f, "khong buoc nao vuot tran du/dt",
          fmt("%.4f <= %.4f", (double)max_step, (double)(SLEW * DT)));
}

/* ========================================================================= */
/* 7. Đổi hệ số lúc đang chạy không giật                                     */
/* ========================================================================= */
static void test_bumpless_gain_change(void)
{
    shark_pid_t p;
    shark_pid_cfg_t c;
    const float E = 5.0f, DT = 0.01f, KI0 = 1.5f, KI1 = 6.0f;
    float u_before, i_before, u_after, jump, one_step, sum_e, naive_jump;
    int i;

    section("7. Doi he so luc dang chay khong giat");

    heater_cfg(&c, 2.0f, KI0, 0.0f);
    shark_pid_init(&p, &c);
    for (i = 0; i < 200; ++i) shark_pid_update(&p, 60.0f, 55.0f, DT);

    u_before = p.output;
    i_before = p.i_term;

    p.cfg.ki = KI1;                             /* tang Ki gap 4 giua chung */
    u_after = shark_pid_update(&p, 60.0f, 55.0f, DT);
    jump = fabsf(u_after - u_before);

    one_step   = KI1 * E * DT;
    sum_e      = i_before / KI0;                /* tong sai so da tich luy */
    naive_jump = fabsf(KI1 * sum_e - KI0 * sum_e);

    printf("   shark: %.4f -> %.4f  nhay %.4f  (mot buoc tich phan = %.4f)\n",
           (double)u_before, (double)u_after, (double)jump, (double)one_step);
    printf("   dang ngay tho i = Ki*tong(e): nhay %.4f\n", (double)naive_jump);

    check(jump <= one_step + 1e-3f, "nhay khong vuot mot buoc tich phan he so moi",
          fmt("%.4f <= %.4f", (double)jump, (double)one_step));
    check(jump * 50.0f < naive_jump, "nho hon dang ngay tho it nhat 50 lan",
          fmt("%.4f vs %.4f", (double)jump, (double)naive_jump));
}

/* ========================================================================= */
/* 8. Phát hiện kẹt, cả hai chiều quay                                       */
/* ========================================================================= */
static void test_stall_detection(void)
{
    shark_pid_t p;
    shark_pid_cfg_t c;
    const float sps[2] = { 100.0f, -100.0f };
    const char *labels[2] = { "chieu duong", "chieu am" };
    int k, i;

    section("8. Phat hien ket co cau");

    for (k = 0; k < 2; ++k) {
        shark_pid_cfg_default(&c);
        c.kp = 10.0f;  c.ki = 0.0f;  c.kd = 0.0f;
        c.out_min = -100.0f;  c.out_max = 100.0f;
        c.d_tau = 0.0f;
        c.stall_time = 0.5f;  c.stall_level = 0.9f;  c.stall_eps = 0.01f;
        shark_pid_init(&p, &c);

        for (i = 0; i < 200; ++i) shark_pid_update(&p, sps[k], 0.0f, 0.01f);

        printf("   %-12s : STALLED = %s\n", labels[k],
               (p.status & SHARK_PID_STALLED) ? "co" : "khong");
        check((p.status & SHARK_PID_STALLED) != 0u, "phat hien duoc ket", labels[k]);
    }

    /* Co phai xoa duoc, khong duoc chot vinh vien */
    shark_pid_clear_status(&p);
    check((p.status & SHARK_PID_STALLED) == 0u, "co ket xoa duoc bang clear_status",
          NULL);

    /* Co cau van chay binh thuong thi khong duoc bao ket */
    shark_pid_cfg_default(&c);
    c.kp = 10.0f;  c.out_min = -100.0f;  c.out_max = 100.0f;  c.d_tau = 0.0f;
    c.stall_time = 0.5f;  c.stall_level = 0.9f;  c.stall_eps = 0.01f;
    shark_pid_init(&p, &c);
    {
        float y = 0.0f;
        for (i = 0; i < 200; ++i) {
            shark_pid_update(&p, 100.0f, y, 0.01f);
            y += 1.0f;                          /* dang chuyen dong that */
        }
    }
    check((p.status & SHARK_PID_STALLED) == 0u,
          "khong bao ket nham khi co cau dang chuyen dong", NULL);
}

/* ========================================================================= */
/* 9. Feedforward                                                            */
/* ========================================================================= */
static void test_feedforward(void)
{
    shark_pid_t p;
    shark_pid_cfg_t c;
    sim_result_t no_ff, with_ff;

    section("9. Feedforward rut ngan thoi gian len");

    heater_cfg(&c, 1.2f, 0.25f, 0.0f);
    shark_pid_init(&p, &c);
    no_ff = sim_shark(&p, 0.01, 120.0, 60.0, 0.0, 100.0);

    c.kf = 0.55f;
    shark_pid_init(&p, &c);
    with_ff = sim_shark(&p, 0.01, 120.0, 60.0, 0.0, 100.0);

    printf("   khong FF: t90=%.2fs  xac lap %.3f\n", no_ff.t90, no_ff.y_final);
    printf("   co FF   : t90=%.2fs  xac lap %.3f\n", with_ff.t90, with_ff.y_final);

    check(with_ff.t90 > 0.0 && with_ff.t90 < no_ff.t90,
          "feedforward rut ngan thoi gian len",
          fmt("%.2fs vs %.2fs", with_ff.t90, no_ff.t90));
    check(fabs(with_ff.y_final - 60.0) < 0.5,
          "feedforward khong lam sai diem xac lap",
          fmt("y=%.4f (%.1f)", with_ff.y_final, 60.0));
}

/* ========================================================================= */
/* 10. Bất biến cơ bản của API                                               */
/* ========================================================================= */
static void test_api_invariants(void)
{
    shark_pid_t p;
    shark_pid_cfg_t c;
    int i;
    int within = 1;

    section("10. Bat bien co ban cua API");

    /* Ngo ra luon nam trong dai, ke ca khi bi kich thich du doi */
    shark_pid_cfg_default(&c);
    c.kp = 1000.0f;  c.ki = 500.0f;  c.kd = 100.0f;
    c.out_min = -42.0f;  c.out_max = 77.0f;
    shark_pid_init(&p, &c);
    for (i = 0; i < 500; ++i) {
        float sp = (i % 7 < 3) ? 1000.0f : -1000.0f;
        float u = shark_pid_update(&p, sp, (float)(i % 11) - 5.0f, 0.01f);
        if (u < -42.0f - 1e-4f || u > 77.0f + 1e-4f) within = 0;
    }
    check(within, "ngo ra luon nam trong [out_min, out_max]", NULL);

    /* reset xoa trang thai nhung giu cau hinh */
    shark_pid_reset(&p);
    check(p.i_term == 0.0f && p.output == 0.0f && p.status == SHARK_PID_OK,
          "reset xoa trang thai chay", NULL);
    check(p.cfg.kp == 1000.0f && p.cfg.out_max == 77.0f,
          "reset giu nguyen cau hinh", NULL);

    /* preload cho chuyen tay -> tu dong */
    shark_pid_cfg_default(&c);
    c.kp = 1.0f;  c.ki = 0.5f;  c.kd = 0.0f;
    c.out_min = 0.0f;  c.out_max = 100.0f;
    c.i_min = 0.0f;  c.i_max = 100.0f;
    shark_pid_init(&p, &c);
    shark_pid_preload(&p, 40.0f);
    {
        /* O trang thai xac lap (e = 0) lenh dau tien phai bam sat 40 */
        float u = shark_pid_update(&p, 50.0f, 50.0f, 0.01f);
        printf("   preload(40) -> lenh dau tien = %.4f\n", (double)u);
        check(fabsf(u - 40.0f) < 0.5f, "preload cho chuyen doi khong giat",
              fmt("%.4f vs %.4f", (double)u, 40.0));
    }

    /* set_output_limits kep lai trang thai hien tai */
    shark_pid_set_output_limits(&p, 0.0f, 10.0f);
    check(p.output <= 10.0f, "set_output_limits kep lai ngo ra hien tai",
          fmt("%.4f <= %.1f", (double)p.output, 10.0));

    /* ki = 0 phai xoa bo tich phan */
    shark_pid_set_gains(&p, 1.0f, 0.0f, 0.0f);
    check(p.i_term == 0.0f, "dat ki = 0 xoa bo tich phan", NULL);

    /* Con tro NULL khong duoc lam sap chuong trinh */
    shark_pid_reset(NULL);
    shark_pid_clear_status(NULL);
    shark_pid_preload(NULL, 1.0f);
    shark_pid_set_gains(NULL, 1.0f, 1.0f, 1.0f);
    shark_pid_set_output_limits(NULL, 0.0f, 1.0f);
    shark_pid_cfg_default(NULL);
    shark_pid_init(NULL, NULL);
    check(shark_pid_update(NULL, 1.0f, 1.0f, 0.01f) == 0.0f,
          "moi ham chiu duoc con tro NULL", NULL);

    /* cfg = NULL phai lay mac dinh */
    shark_pid_init(&p, NULL);
    check(p.cfg.c == 0.0f && p.cfg.b == 1.0f,
          "init(cfg = NULL) dung cau hinh mac dinh", NULL);
}

/* ========================================================================= */
/* 11. Bẫy b < 1: khâu I phải gánh Kp*r*(1-b)                                */
/* ========================================================================= */
static void test_setpoint_weighting_trap(void)
{
    shark_pid_t p;
    shark_pid_cfg_t c;
    const float KP = 4.0f, B = 0.7f;
    const double R = 60.0;
    float i_b1, i_b07, diff, predicted;
    sim_result_t r1, r2;

    section("11. Bay b < 1 (khau I phai ganh them Kp*r*(1-b))");

    /* Phai chay VONG KIN: ep measurement = setpoint thi sai so bang 0 nen
       khau I khong bao gio tich luy, test se vo nghia. */
    shark_pid_cfg_default(&c);
    c.kp = KP;  c.ki = 1.5f;  c.kd = 0.0f;
    c.c = 0.0f;
    c.out_min = -200.0f;  c.out_max = 200.0f;
    c.i_min   = -200.0f;  c.i_max   = 200.0f;
    c.d_tau = 0.0f;
    c.flags = SHARK_PID_F_TRAPEZOID_I | SHARK_PID_F_CLAMP_I;

    c.b = 1.0f;
    shark_pid_init(&p, &c);
    r1 = sim_shark(&p, 0.01, 300.0, R, -200.0, 200.0);
    i_b1 = p.i_term;
    printf("   b=1.0 -> i_term=%8.4f  p_term=%8.4f  u=%8.4f  y=%8.4f\n",
           (double)i_b1, (double)p.p_term, (double)p.output, r1.y_final);

    c.b = B;
    shark_pid_init(&p, &c);
    r2 = sim_shark(&p, 0.01, 300.0, R, -200.0, 200.0);
    i_b07 = p.i_term;
    printf("   b=0.7 -> i_term=%8.4f  p_term=%8.4f  u=%8.4f  y=%8.4f\n",
           (double)i_b07, (double)p.p_term, (double)p.output, r2.y_final);

    diff = i_b07 - i_b1;
    predicted = KP * (float)R * (1.0f - B);
    printf("   chenh lech i_term = %.4f   ly thuyet Kp*r*(1-b) = %.4f\n",
           (double)diff, (double)predicted);

    check(fabs(r1.y_final - R) < 0.1 && fabs(r2.y_final - R) < 0.1,
          "ca hai deu xac lap dung dich", NULL);
    check(fabsf(diff - predicted) < 0.5f,
          "khau I phai ganh them dung Kp*r*(1-b) khi ha b",
          fmt("%.4f vs %.4f", (double)diff, (double)predicted));
    check(fabsf(p.p_term + predicted) < 0.5f,
          "o xac lap khau P xuat ra dung -Kp*r*(1-b)",
          fmt("%.4f vs %.4f", (double)p.p_term, (double)(-predicted)));
}

/* ========================================================================= */
/* main                                                                      */
/* ========================================================================= */
int main(void)
{
    printf("shark_pid " SHARK_PID_VERSION_STR " - bo test\n");
    printf("kich thuoc: shark_pid_t = %u byte, cfg = %u byte\n",
           (unsigned)sizeof(shark_pid_t), (unsigned)sizeof(shark_pid_cfg_t));

    test_sample_rate_independence();
    test_antiwindup();
    test_derivative_kick();
    test_deadband_no_fake_derivative();
    test_nan_immunity();
    test_slew_limit();
    test_bumpless_gain_change();
    test_stall_detection();
    test_feedforward();
    test_api_invariants();
    test_setpoint_weighting_trap();

    printf("\n========================================\n");
    printf("dat %d, hong %d, tong %d\n", g_pass, g_fail, g_pass + g_fail);
    printf("========================================\n");

    if (g_fail != 0) {
        printf("CO TEST HONG\n");
        return 1;
    }
    printf("TAT CA DAT\n");
    return 0;
}
