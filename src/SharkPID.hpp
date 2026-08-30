/**
 * @file    SharkPID.hpp
 * @brief   Vỏ bọc C++ mỏng cho shark_pid — dành cho Arduino / ESP32 / PlatformIO.
 *          Toàn bộ thuật toán vẫn nằm ở lõi C99; file này chỉ thêm cú pháp.
 * @author  Trần Nguyên Bình (github.com/nguyenbinh-shark)
 * @license MIT
 */
#ifndef SHARK_PID_HPP
#define SHARK_PID_HPP

#include "shark_pid.h"

#if defined(ARDUINO)
#include <Arduino.h>
#endif

/**
 * Dùng nhanh:
 *
 *   SharkPID pid(2.0f, 0.35f, 0.8f, 0.0f, 255.0f);
 *   float u = pid.update(setpoint, analogRead(A0));   // tự đo dt bằng micros()
 *
 * Hoặc tự cấp dt (khuyến nghị khi chạy trong task RTOS chu kỳ cố định):
 *
 *   float u = pid.update(setpoint, measurement, 0.001f);
 *
 * Mọi tham số ánh xạ 1-1 với một ô trong khối `PID Controller (2DOF)` của
 * Simulink — xem extras/Test_Shark_PID/README.md để biết bảng ánh xạ.
 */
class SharkPID {
public:
    SharkPID()
    {
        shark_pid_init(&core_, 0);
        initStamp();
    }

    SharkPID(float kp, float ki, float kd,
             float outMin = -255.0f, float outMax = 255.0f)
    {
        shark_pid_cfg_t cfg;
        shark_pid_cfg_default(&cfg);
        cfg.kp = kp;
        cfg.ki = ki;
        cfg.kd = kd;
        cfg.out_min = outMin;
        cfg.out_max = outMax;
        cfg.i_min = outMin;     /* mặc định: trần tích phân = trần ngõ ra */
        cfg.i_max = outMax;
        shark_pid_init(&core_, &cfg);
        initStamp();
    }

    explicit SharkPID(const shark_pid_cfg_t &cfg)
    {
        shark_pid_init(&core_, &cfg);
        initStamp();
    }

    /* ---------------- Tính toán ---------------- */

    /** Chu kỳ lấy mẫu do người gọi cung cấp, đơn vị giây. */
    float update(float setpoint, float measurement, float dt)
    {
        return shark_pid_update(&core_, setpoint, measurement, dt);
    }

    /** Gọi như một hàm toán học, kiểu functor giống SimpleFOC. */
    float operator()(float setpoint, float measurement, float dt)
    {
        return shark_pid_update(&core_, setpoint, measurement, dt);
    }

#if defined(ARDUINO)
    /**
     * Tự đo dt bằng micros().
     *
     * Phép trừ dưới đây là số nguyên KHÔNG DẤU, nên nó vẫn cho đúng khoảng
     * cách khi bộ đếm micros() tràn 32-bit (~71,6 phút). Bộ bảo vệ dt trong
     * lõi là để chặn nhịp đầu tiên và những lần lịch trình bị treo, chứ
     * không phải để chữa tràn số.
     */
    float update(float setpoint, float measurement)
    {
        uint32_t now = micros();
        float dt;
        if (!hasStamp_) {
            dt = core_.cfg.dt_nominal;
            hasStamp_ = true;
        } else {
            dt = (float)(uint32_t)(now - stamp_) * 1e-6f;
        }
        stamp_ = now;
        return shark_pid_update(&core_, setpoint, measurement, dt);
    }

    float operator()(float setpoint, float measurement)
    {
        return update(setpoint, measurement);
    }
#endif

    /* ---------------- Điều khiển vòng đời ---------------- */

    void reset()
    {
        shark_pid_reset(&core_);
        initStamp();
    }

    void setGains(float kp, float ki, float kd)
    {
        shark_pid_set_gains(&core_, kp, ki, kd);
    }

    void setOutputLimits(float outMin, float outMax)
    {
        shark_pid_set_output_limits(&core_, outMin, outMax);
    }

    void setIntegralLimits(float iMin, float iMax)
    {
        core_.cfg.i_min = iMin;
        core_.cfg.i_max = iMax;
    }

    /** Ô Filter coefficient (N) của khối PID, rad/giây. <= 0 = bỏ lọc khâu D. */
    void setFilterN(float n)
    {
        core_.cfg.n = n;
    }

    /** Chuyển tay -> tự động không giật. */
    void preload(float outputNow)
    {
        shark_pid_preload(&core_, outputNow);
        initStamp();
    }

    /* ---------------- Đọc trạng thái ---------------- */

    float output() const { return core_.output; }
    float error()  const { return core_.error; }
    float pTerm()  const { return core_.p_term; }
    float iTerm()  const { return core_.i_term; }
    float dTerm()  const { return core_.d_term; }
    float dtUsed() const { return core_.dt_used; }

    uint32_t status() const { return core_.status; }
    bool isSaturated() const { return (core_.status & SHARK_PID_SATURATED) != 0u; }
    bool hadBadInput() const { return (core_.status & SHARK_PID_BAD_INPUT) != 0u; }
    bool hadBadDt()    const { return (core_.status & SHARK_PID_BAD_DT) != 0u; }

    /** Truy cập thẳng cấu hình để chỉnh chi tiết: pid.cfg().n = 50.0f; */
    shark_pid_cfg_t &cfg() { return core_.cfg; }
    shark_pid_t &core() { return core_; }

private:
    void initStamp()
    {
#if defined(ARDUINO)
        stamp_ = 0;
        hasStamp_ = false;
#endif
    }

    shark_pid_t core_;
#if defined(ARDUINO)
    uint32_t stamp_;
    bool hasStamp_;
#endif
};

#endif /* SHARK_PID_HPP */
