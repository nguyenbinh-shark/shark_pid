/**
 * @file    shark_pid.h
 * @brief   shark_pid — bộ điều khiển PID nhúng, là bản sao số của khối
 *          `PID Controller (2DOF)` trong MATLAB/Simulink.
 *          Embedded 2-DOF PID controller, faithful to the Simulink block.
 * @author  Trần Nguyên Bình (github.com/nguyenbinh-shark)
 * @version 2.0.0
 * @license MIT
 *
 * Thiết kế theo 4 nguyên tắc:
 *   1. MỖI tham số ánh xạ 1-1 với một ô trong Block Parameters của khối
 *      `PID Controller (2DOF)`. Không giữ tính năng nào khối không có, vì thứ
 *      không mô phỏng được thì không kiểm chứng được trước khi nạp firmware.
 *   2. dt là tham số tường minh -> đổi tần số vòng lặp không làm trôi P/I/D.
 *   3. Không phụ thuộc Arduino/HAL -> biên được trên STM32, ESP-IDF, PC, unit test.
 *   4. Không bao giờ trả về NaN/Inf, không bao giờ chia cho 0.
 *
 * Phương trình cài đặt (Form = Parallel, Time domain = Discrete-time):
 *
 *   u = P*(b*r - y) + I*Fi(z)*(r - y) + Dbranch(z)*(c*r - y)
 *
 *   Fi(z)   = Ts*z/(z-1)          Backward Euler   (cờ TRAPEZOID_I tắt)
 *           = (Ts/2)*(z+1)/(z-1)  Trapezoidal      (cờ TRAPEZOID_I bật)
 *   Dbranch = D*(z-1)/(Ts*z)                       (n <= 0, không lọc)
 *           = D*N/(1 + N*Ts*z/(z-1))               (n  > 0, Filter method =
 *                                                   Backward Euler)
 *
 * Bảng ánh xạ đầy đủ sang từng ô trong hộp thoại: xem extras/Test_Shark_PID/README.md.
 */
#ifndef SHARK_PID_H
#define SHARK_PID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SHARK_PID_VERSION_MAJOR 2
#define SHARK_PID_VERSION_MINOR 0
#define SHARK_PID_VERSION_PATCH 0
#define SHARK_PID_VERSION_STR   "2.0.0"

/* ------------------------------------------------------------------------- */
/* Cờ chế độ (behaviour flags)                                               */
/*                                                                           */
/* Chỉ những LỰA CHỌN HÀNH VI mới nằm ở đây — đúng những ô kiểu combo box     */
/* của khối. Tham số có độ lớn (bộ lọc, cận tích phân) tự tắt khi bằng 0.     */
/* ------------------------------------------------------------------------- */
typedef enum {
    SHARK_PID_F_NONE        = 0u,
    /** `Integrator method` = Trapezoidal. Tắt cờ = Backward Euler. */
    SHARK_PID_F_TRAPEZOID_I = 1u << 0,
    /** `Anti-windup method` = clamping. */
    SHARK_PID_F_CLAMP_I     = 1u << 1,
    /** `Anti-windup method` = back-calculation (dùng cfg.kb). */
    SHARK_PID_F_BACKCALC_I  = 1u << 2
} shark_pid_flag_t;

/* Khối chỉ chọn được MỘT phương pháp chống windup. Bật cả hai cờ thì clamping
   thắng — đúng như shark_pid_map2simulink() ánh xạ. */

/* ------------------------------------------------------------------------- */
/* Trạng thái (bitmask — nhiều cờ có thể cùng xảy ra)                        */
/*                                                                           */
/* Khối Simulink không có ngõ ra trạng thái; phần này là của riêng firmware   */
/* và không đụng vào giá trị u, nên hai bên vẫn khớp tuyệt đối. Mọi cờ đều    */
/* tính lại mỗi chu kỳ, không cờ nào chốt lại.                                */
/* ------------------------------------------------------------------------- */
typedef enum {
    SHARK_PID_OK        = 0u,
    /** Ngõ ra đang bị cắt biên. */
    SHARK_PID_SATURATED = 1u << 0,
    /** setpoint/measurement là NaN hoặc Inf -> chu kỳ này bị bỏ qua. */
    SHARK_PID_BAD_INPUT = 1u << 1,
    /** dt bất thường -> đã thay bằng cfg.dt_nominal. */
    SHARK_PID_BAD_DT    = 1u << 2
} shark_pid_status_t;

/* ------------------------------------------------------------------------- */
/* Cấu hình — mỗi trường là một ô trong Block Parameters                     */
/* ------------------------------------------------------------------------- */
typedef struct {
    /* --- Tab Main: hệ số --- */
    float kp;           /**< Ô `P`. */
    float ki;           /**< Ô `I`, đơn vị 1/giây (dt được nhân bên trong). */
    float kd;           /**< Ô `D`, đơn vị giây (dt được chia bên trong). */

    /* --- Tab Main: setpoint weighting — đây chính là phần "2DOF" --- */
    float b;            /**< Ô `Setpoint weight (b)`: 1 = P cổ điển, 0 = P-on-measurement. */
    float c;            /**< Ô `Setpoint weight (c)`: 0 = D-on-measurement (nên dùng), 1 = D-on-error. */

    /* --- Tab Main: bộ lọc khâu D --- */
    float n;            /**< Ô `Filter coefficient (N)`, đơn vị rad/giây.
                             <= 0 = bỏ tích `Use filtered derivative`.
                             Hằng số thời gian tương đương là 1/N giây. */

    /* --- Tab Saturation: cận ngõ ra (`Limit output`) --- */
    float out_min;      /**< `Lower saturation limit`. Cơ cấu 2 chiều: đặt giá trị âm. */
    float out_max;      /**< `Upper saturation limit`. */

    /* --- Tab Saturation: cận khâu tích phân (`Limit integrator`) --- */
    float i_min;        /**< `Lower integrator saturation limit`. */
    float i_max;        /**< `Upper integrator saturation limit`. */

    /* --- Tab Saturation: chống windup --- */
    float kb;           /**< Ô `Back-calculation coefficient (Kb)`, đơn vị 1/giây.
                             Chỉ có tác dụng khi bật SHARK_PID_F_BACKCALC_I. */

    /* --- Bảo vệ nhịp lấy mẫu (khối KHÔNG có: Ts của nó là hằng số) --- */
    float dt_max;       /**< dt lớn hơn ngưỡng này coi là bất thường. */
    float dt_nominal;   /**< dt thay thế khi dt bất thường; cũng là `Sample time` của khối. */

    uint32_t flags;     /**< Tổ hợp shark_pid_flag_t. */
} shark_pid_cfg_t;

/* ------------------------------------------------------------------------- */
/* Đối tượng điều khiển                                                      */
/* ------------------------------------------------------------------------- */
typedef struct {
    shark_pid_cfg_t cfg;    /**< Sửa trực tiếp lúc đang chạy cũng được. */

    /* --- Chỉ đọc: rất hữu ích khi vẽ đồ thị / gỡ lỗi --- */
    float p_term;           /**< Đóng góp của khâu P ở chu kỳ vừa rồi. */
    float i_term;           /**< Đóng góp của khâu I — NGÕ RA bộ tích phân, đã nhân Ki. */
    float d_term;           /**< Đóng góp của khâu D (sau lọc). */
    float error;            /**< Sai số r - y của chu kỳ vừa rồi. */
    float output;           /**< Lệnh điều khiển vừa xuất. */
    float dt_used;          /**< dt thực sử dụng sau khi qua bộ bảo vệ. */
    uint32_t status;        /**< Tổ hợp shark_pid_status_t. */

    /* --- Nội bộ: đúng hai biến trạng thái của khối --- */
    float i_state;          /**< Trạng thái Discrete-Time Integrator. ĐÃ BAO GỒM Ki
                                 -> đổi Ki lúc chạy không làm giật ngõ ra. */
    float d_state;          /**< Trạng thái bộ lọc khâu D (`Filter Initial condition`). */
    float prev_d_input;     /**< Chỉ dùng khi n <= 0 (khâu D không lọc). */
    uint8_t primed;
} shark_pid_t;

/* ------------------------------------------------------------------------- */
/* API                                                                       */
/* ------------------------------------------------------------------------- */

/**
 * Nạp cấu hình mặc định an toàn: D-on-measurement, chống windup kiểu kẹp,
 * I hình thang, lọc D với N = 100 (tương đương hằng số thời gian 10 ms),
 * dải ngõ ra ±100, dt danh định 1 ms.
 */
void shark_pid_cfg_default(shark_pid_cfg_t *cfg);

/** Khởi tạo. Truyền cfg = NULL để dùng cấu hình mặc định. */
void shark_pid_init(shark_pid_t *pid, const shark_pid_cfg_t *cfg);

/** Xoá toàn bộ trạng thái chạy, giữ nguyên cấu hình. */
void shark_pid_reset(shark_pid_t *pid);

/** Đổi hệ số lúc đang chạy. Không làm giật ngõ ra (i_state đã chứa sẵn Ki). */
void shark_pid_set_gains(shark_pid_t *pid, float kp, float ki, float kd);

/** Đổi dải ngõ ra và kẹp lại trạng thái hiện tại cho khớp. */
void shark_pid_set_output_limits(shark_pid_t *pid, float out_min, float out_max);

/**
 * Chuyển tay -> tự động không giật (bumpless transfer).
 * Nạp trước bộ tích phân để lệnh đầu tiên bám sát @p output_now.
 * Chính xác tuyệt đối khi P/D xấp xỉ 0 tại thời điểm chuyển — tức là khi hệ
 * đang ở trạng thái xác lập, đúng như tình huống chuyển chế độ thực tế.
 */
void shark_pid_preload(shark_pid_t *pid, float output_now);

/**
 * Tính một chu kỳ.
 * @param dt Chu kỳ lấy mẫu thực tế, đơn vị GIÂY. Đo bằng micros()/DWT/timer.
 *           dt <= 0, NaN, hoặc > cfg.dt_max sẽ bị thay bằng cfg.dt_nominal.
 * @return   Lệnh điều khiển, luôn nằm trong [cfg.out_min, cfg.out_max].
 */
float shark_pid_update(shark_pid_t *pid, float setpoint, float measurement, float dt);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* SHARK_PID_H */
