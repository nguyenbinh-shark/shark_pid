/**
 * 02_Motor_Bidirectional — Điều khiển vị trí động cơ DC hai chiều qua cầu H.
 *
 * Đây chính là trường hợp mà kiểu kẹp ngõ ra [0, max] KHÔNG dùng được:
 * động cơ phải quay được cả hai chiều nên out_min phải âm.
 *
 * Đặc điểm bài toán:
 *   - Encoder sạch hơn cảm biến nhiệt nhưng vi phân vẫn khuếch đại nhiễu.
 *   - Setpoint thay đổi theo bậc -> bắt buộc D-on-measurement (cfg.c = 0).
 *   - Hộp số sợ sốc dòng -> giới hạn dốc du/dt.
 *   - Cần biết khi cơ cấu bị chèn cứng -> bật phát hiện kẹt.
 *
 * Bài học tương ứng trên blog:
 *   https://nguyenbinh-shark.github.io/posts/2026/08/pid-code-5-doc-pid-nguoi-di-lam/
 */
#include <SharkPID.hpp>

const uint8_t PIN_PWM  = 5;
const uint8_t PIN_DIR_A = 6;
const uint8_t PIN_DIR_B = 7;

volatile long encoderTicks = 0;
const float TICKS_PER_DEG = 11.38f;

SharkPID pid;

float readAngleDeg()
{
    noInterrupts();
    long t = encoderTicks;
    interrupts();
    return (float)t / TICKS_PER_DEG;
}

void driveMotor(float u)                // u ∈ [-255, 255]
{
    bool forward = (u >= 0.0f);
    digitalWrite(PIN_DIR_A, forward ? HIGH : LOW);
    digitalWrite(PIN_DIR_B, forward ? LOW : HIGH);

    float mag = forward ? u : -u;
    if (mag > 255.0f) mag = 255.0f;
    analogWrite(PIN_PWM, (int)(mag + 0.5f));
}

void setup()
{
    Serial.begin(115200);
    pinMode(PIN_PWM, OUTPUT);
    pinMode(PIN_DIR_A, OUTPUT);
    pinMode(PIN_DIR_B, OUTPUT);

    shark_pid_cfg_t cfg;
    shark_pid_cfg_default(&cfg);

    cfg.kp = 6.0f;
    cfg.ki = 2.5f;
    cfg.kd = 0.15f;

    cfg.b = 1.0f;                       // P bám setpoint bình thường
    cfg.c = 0.0f;                       // D chỉ nhìn encoder -> hết derivative kick

    cfg.out_min = -255.0f;              // <<< HAI CHIỀU
    cfg.out_max =  255.0f;
    cfg.i_min   = -120.0f;              // trần tích phân hẹp hơn trần ngõ ra
    cfg.i_max   =  120.0f;

    cfg.d_tau    = 0.008f;              // lọc D 8 ms
    cfg.out_slew = 1500.0f;             // tối đa 1500 đơn vị PWM mỗi giây
    cfg.deadband = 0.5f;                // ±0.5° thì thôi rung

    // Phát hiện kẹt: ra lệnh trên 90% công suất mà trục quay chậm hơn
    // 2 °/s liên tục 0.4 s -> nghi bị chèn.
    cfg.stall_time  = 0.4f;
    cfg.stall_level = 0.90f;
    cfg.stall_eps   = 2.0f;

    cfg.dt_nominal = 0.002f;            // 500 Hz

    cfg.flags = SHARK_PID_F_TRAPEZOID_I
              | SHARK_PID_F_CLAMP_I
              | SHARK_PID_F_STALL_CUTOFF;   // tự ngắt để cứu hộp số

    pid.core().cfg = cfg;
    pid.reset();
}

void loop()
{
    static uint32_t last = 0;
    static float target = 90.0f;

    if (micros() - last >= 2000UL) {    // 500 Hz
        last = micros();

        float angle = readAngleDeg();
        float u = pid.update(target, angle);

        driveMotor(u);

        if (pid.isStalled()) {
            Serial.println(F("!! KET CO CAU - da ngat cong suat"));
            // Sau khi xử lý xong nguyên nhân:
            //   pid.clearStatus();
            //   pid.preload(0.0f);     // vào lại chế độ tự động không giật
        }
    }
}
