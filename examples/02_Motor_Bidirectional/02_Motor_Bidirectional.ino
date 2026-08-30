/**
 * 02_Motor_Bidirectional — Điều khiển vị trí động cơ DC hai chiều qua cầu H.
 *
 * Đây chính là trường hợp mà kiểu kẹp ngõ ra [0, max] KHÔNG dùng được:
 * động cơ phải quay được cả hai chiều nên out_min phải âm.
 *
 * Đặc điểm bài toán:
 *   - Encoder sạch hơn cảm biến nhiệt nhưng vi phân vẫn khuếch đại nhiễu.
 *   - Setpoint thay đổi theo bậc -> bắt buộc D-on-measurement (cfg.c = 0).
 *   - Cơ cấu bão hoà -> chống windup kiểu back-calculation cho mượt.
 *
 * Những thứ nằm NGOÀI khối PID (2DOF) — giới hạn dốc du/dt, vùng chết,
 * phát hiện kẹt — không nằm trong thư viện, đúng như trong Simulink chúng
 * là khối Rate Limiter / Dead Zone / logic riêng nối SAU khối PID. Muốn có
 * thì viết vài dòng ở tầng gọi, xem hàm applySlew() bên dưới.
 *
 * Bài học tương ứng trên blog:
 *   https://nguyenbinh-shark.github.io/posts/2026/08/pid-code-5-doc-pid-nguoi-di-lam/
 */
#include <SharkPID.hpp>

const uint8_t PIN_PWM  = 5;
const uint8_t PIN_DIR_A = 6;
const uint8_t PIN_DIR_B = 7;

const float CTRL_DT   = 0.002f;         // 500 Hz
const float OUT_SLEW  = 1500.0f;        // đơn vị PWM mỗi giây

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

/**
 * Giới hạn dốc du/dt — tương đương một khối `Rate Limiter` nối sau khối PID
 * trong Simulink. Để ngoài lõi cho thư viện đúng bằng khối PID, và cũng vì
 * hộp số của bạn mới là thứ quyết định con số này.
 */
float applySlew(float u, float dt)
{
    static float prev = 0.0f;
    float maxStep = OUT_SLEW * dt;
    float delta = u - prev;

    if (delta >  maxStep) u = prev + maxStep;
    if (delta < -maxStep) u = prev - maxStep;

    prev = u;
    return u;
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

    cfg.n = 125.0f;                     // Filter coefficient N (~ 8 ms)

    cfg.dt_nominal = CTRL_DT;

    // Back-calculation mượt hơn kiểu kẹp cứng. Kb lớn thì chống windup mạnh
    // hơn; khởi điểm ~ ki/kp rồi tăng dần đến khi vọt lố hết cải thiện.
    cfg.kb    = 3.0f;
    cfg.flags = SHARK_PID_F_TRAPEZOID_I | SHARK_PID_F_BACKCALC_I;

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

        driveMotor(applySlew(u, CTRL_DT));

        if (pid.isSaturated()) {
            // Ra lệnh hết công suất mà trục không nhúc nhích -> nghi kẹt.
            // Logic đó thuộc về tầng ứng dụng, không phải bộ điều khiển.
            Serial.println(F("[BAO HOA]"));
        }
    }
}
