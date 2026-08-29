/**
 * @file    shark_pid.h
 * @brief   shark_pid — bộ điều khiển PID nhúng, độc lập tần số lấy mẫu.
 *          Embedded PID controller, sample-rate independent.
 * @author  Trần Nguyên Bình (github.com/nguyenbinh-shark)
 * @version 1.0.0
 * @license MIT
 *
 * Thiết kế theo 4 nguyên tắc:
 *   1. dt là tham số tường minh -> đổi tần số vòng lặp không làm trôi Kp/Ki/Kd.
 *   2. Không phụ thuộc Arduino/HAL -> biên được trên STM32, ESP-IDF, PC, unit test.
 *   3. Tham số tự tắt: đặt 0 là tính năng tắt, không cần nhớ bật cờ.
 *   4. Không bao giờ trả về NaN/Inf, không bao giờ chia cho 0.
 */
#ifndef SHARK_PID_H
#define SHARK_PID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SHARK_PID_VERSION_MAJOR 1
#define SHARK_PID_VERSION_MINOR 0
#define SHARK_PID_VERSION_PATCH 0
#define SHARK_PID_VERSION_STR   "1.0.0"

/* ------------------------------------------------------------------------- */
/* Cờ chế độ (behaviour flags)                                               */
/*                                                                           */
/* Chỉ những LỰA CHỌN HÀNH VI mới nằm ở đây. Các tính năng có độ lớn (lọc,   */
/* deadband, slew, feedforward...) tự tắt khi tham số = 0, nên không cần cờ.  */
/* ------------------------------------------------------------------------- */
typedef enum {
    SHARK_PID_F_NONE          = 0u,
    /** Tích phân hình thang (Tustin) thay cho hình chữ nhật. */
    SHARK_PID_F_TRAPEZOID_I   = 1u << 0,
    /** Chống windup kiểu kẹp có điều kiện: bão hoà thì hoàn tác bước tích phân. */
    SHARK_PID_F_CLAMP_I       = 1u << 1,
    /** Chống windup kiểu back-calculation: kéo tích phân về theo lượng bị cắt (cần cfg.kt). */
    SHARK_PID_F_BACKCALC_I    = 1u << 2,
    /** Trong vùng chết: giữ nguyên lệnh cũ thay vì tiếp tục tính với e = 0. */
    SHARK_PID_F_DEADBAND_HOLD = 1u << 3,
    /** Khi phát hiện kẹt cơ cấu: tự ép ngõ ra về 0 (mặc định chỉ báo trạng thái). */
    SHARK_PID_F_STALL_CUTOFF  = 1u << 4
} shark_pid_flag_t;

/* ------------------------------------------------------------------------- */
/* Trạng thái (bitmask — nhiều cờ có thể cùng xảy ra)                        */
/* ------------------------------------------------------------------------- */
typedef enum {
    SHARK_PID_OK        = 0u,
    /** Ngõ ra đang bị cắt biên. */
    SHARK_PID_SATURATED = 1u << 0,
    /** Cơ cấu nghi bị kẹt (xoá bằng shark_pid_clear_status). */
    SHARK_PID_STALLED   = 1u << 1,
    /** setpoint/measurement là NaN hoặc Inf -> chu kỳ này bị bỏ qua. */
    SHARK_PID_BAD_INPUT = 1u << 2,
    /** dt bất thường -> đã thay bằng cfg.dt_nominal. */
    SHARK_PID_BAD_DT    = 1u << 3
} shark_pid_status_t;

/* ------------------------------------------------------------------------- */
/* Cấu hình                                                                  */
/* ------------------------------------------------------------------------- */
typedef struct {
    /* --- Hệ số cơ bản --- */
    float kp;           /**< Hệ số tỉ lệ. */
    float ki;           /**< Hệ số tích phân, đơn vị 1/giây (đã nhân dt bên trong). */
    float kd;           /**< Hệ số vi phân, đơn vị giây (đã chia dt bên trong). */

    /* --- Setpoint weighting (PID 2 bậc tự do / 2-DOF) --- */
    float b;            /**< Trọng số setpoint khâu P: 1 = cổ điển, 0 = P-on-measurement. */
    float c;            /**< Trọng số setpoint khâu D: 0 = D-on-measurement (nên dùng), 1 = D-on-error. */

    /* --- Feedforward --- */
    float kf;           /**< Bù tĩnh:    kf * setpoint. */
    float kf_dot;       /**< Bù vận tốc: kf_dot * d(setpoint)/dt. */

    /* --- Giới hạn --- */
    float out_min;      /**< Cận dưới ngõ ra. Động cơ 2 chiều: đặt giá trị âm. */
    float out_max;      /**< Cận trên ngõ ra. */
    float i_min;        /**< Cận dưới khâu tích phân. */
    float i_max;        /**< Cận trên khâu tích phân. */
    float kt;           /**< Hệ số back-calculation, đơn vị 1/giây. Khởi điểm: 1/kp. */

    /* --- Vùng chết --- */
    float deadband;     /**< |e| < deadband thì P và I ngừng tác động. 0 = tắt. */

    /* --- Biến tốc độ tích phân (Changing Integral Rate) --- */
    float ci_a;         /**< Bề rộng vùng chuyển tiếp. <= 0 = tắt. */
    float ci_b;         /**< Ngưỡng mở tích phân 100%. */

    /* --- Bộ lọc: khai báo bằng HẰNG SỐ THỜI GIAN (giây), không phải alpha --- */
    float d_tau;        /**< Hằng số thời gian lọc khâu D. <= 0 = tắt. */
    float out_tau;      /**< Hằng số thời gian lọc ngõ ra. <= 0 = tắt. Lưu ý: thêm trễ pha trong vòng kín. */

    /* --- Giới hạn dốc ngõ ra --- */
    float out_slew;     /**< |du/dt| tối đa, đơn vị ngõ ra trên giây. <= 0 = tắt. */

    /* --- Bảo vệ nhịp lấy mẫu --- */
    float dt_max;       /**< dt lớn hơn ngưỡng này coi là bất thường. */
    float dt_nominal;   /**< dt thay thế khi dt bất thường. */

    /* --- Phát hiện kẹt cơ cấu --- */
    float stall_time;   /**< Số giây liên tục để báo kẹt. <= 0 = tắt. */
    float stall_level;  /**< Tỉ lệ so với biên ngõ ra để coi là "đã hết công suất", 0..1. */
    float stall_eps;    /**< |d(measure)/dt| dưới ngưỡng này coi như đứng yên. Đơn vị đo trên giây. */

    uint32_t flags;     /**< Tổ hợp shark_pid_flag_t. */
} shark_pid_cfg_t;

/* ------------------------------------------------------------------------- */
/* Đối tượng điều khiển                                                      */
/* ------------------------------------------------------------------------- */
typedef struct {
    shark_pid_cfg_t cfg;    /**< Sửa trực tiếp lúc đang chạy cũng được. */

    /* --- Chỉ đọc: rất hữu ích khi vẽ đồ thị / gỡ lỗi --- */
    float p_term;           /**< Đóng góp của khâu P ở chu kỳ vừa rồi. */
    float i_term;           /**< Bộ tích luỹ. ĐÃ BAO GỒM Ki -> đổi Ki lúc chạy không làm giật ngõ ra. */
    float d_term;           /**< Đóng góp của khâu D (sau lọc). */
    float ff_term;          /**< Đóng góp của feedforward. */
    float error;            /**< Sai số thô của chu kỳ vừa rồi. */
    float output;           /**< Lệnh điều khiển vừa xuất. */
    float dt_used;          /**< dt thực sử dụng sau khi qua bộ bảo vệ. */
    uint32_t status;        /**< Tổ hợp shark_pid_status_t. */

    /* --- Nội bộ --- */
    float prev_error;
    float prev_d_input;
    float prev_setpoint;
    float prev_meas;
    float prev_output;
    float d_filt;
    float out_filt;
    float stall_timer;
    uint8_t primed;
} shark_pid_t;

/* ------------------------------------------------------------------------- */
/* API                                                                       */
/* ------------------------------------------------------------------------- */

/**
 * Nạp cấu hình mặc định an toàn: D-on-measurement, kẹp chống windup,
 * tích phân hình thang, lọc D 10 ms, dải ngõ ra ±100, dt danh định 1 ms.
 */
void shark_pid_cfg_default(shark_pid_cfg_t *cfg);

/** Khởi tạo. Truyền cfg = NULL để dùng cấu hình mặc định. */
void shark_pid_init(shark_pid_t *pid, const shark_pid_cfg_t *cfg);

/** Xoá toàn bộ trạng thái chạy, giữ nguyên cấu hình. */
void shark_pid_reset(shark_pid_t *pid);

/** Đổi hệ số lúc đang chạy. Không làm giật ngõ ra (i_term đã chứa sẵn Ki). */
void shark_pid_set_gains(shark_pid_t *pid, float kp, float ki, float kd);

/** Đổi dải ngõ ra và kẹp lại trạng thái hiện tại cho khớp. */
void shark_pid_set_output_limits(shark_pid_t *pid, float out_min, float out_max);

/**
 * Chuyển tay -> tự động không giật (bumpless transfer).
 * Nạp trước bộ tích phân để lệnh đầu tiên bám sát @p output_now.
 * Chính xác tuyệt đối khi P/D/FF xấp xỉ 0 tại thời điểm chuyển — tức là
 * khi hệ đang ở trạng thái xác lập, đúng như tình huống chuyển chế độ thực tế.
 */
void shark_pid_preload(shark_pid_t *pid, float output_now);

/** Xoá cờ lỗi đã chốt (đặc biệt là SHARK_PID_STALLED). */
void shark_pid_clear_status(shark_pid_t *pid);

/**
 * Tính một chu kỳ.
 * @param dt Chu kỳ lấy mẫu thực tế, đơn vị GIÂY. Đo bằng micros()/DWT/timer.
 *           dt <= 0, NaN, hoặc > cfg.dt_max sẽ bị thay bằng cfg.dt_nominal.
 * @return   Lệnh điều khiển, luôn nằm trong [cfg.out_min, cfg.out_max].
 */
float shark_pid_update(shark_pid_t *pid, float setpoint, float measurement, float dt);

/**
 * Như shark_pid_update nhưng cộng thêm một lượng feedforward tính từ bên ngoài
 * (bù trọng lực cánh tay robot, bù ma sát tĩnh, bù điện áp nguồn...).
 */
float shark_pid_update_ff(shark_pid_t *pid, float setpoint, float measurement,
                          float dt, float ff_extra);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* SHARK_PID_H */
