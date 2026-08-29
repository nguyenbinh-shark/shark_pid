/**
 * 01_Heater_Basic — Điều khiển nhiệt độ lò sấy (cơ cấu một chiều).
 *
 * Đặc điểm bài toán:
 *   - Sò công suất chỉ nung được, không làm lạnh -> ngõ ra [0, 255].
 *   - Cảm biến nhiệt có nhiễu -> cần lọc khâu D.
 *   - Quán tính nhiệt lớn -> rất dễ windup khi khởi động nguội.
 *
 * Bài học tương ứng trên blog:
 *   https://nguyenbinh-shark.github.io/posts/2026/08/pid-code-2-cac-benh-pid-dau-tay/
 */
#include <SharkPID.hpp>

const uint8_t HEATER_PIN = 9;
const uint8_t SENSOR_PIN = A0;

const float SETPOINT_C = 60.0f;
const uint32_t PERIOD_MS = 50;          // 20 Hz là quá đủ cho quán tính nhiệt

SharkPID pid;

float readTemperatureC()
{
    // Thay bằng cảm biến thật của bạn (NTC, MAX6675, DS18B20...).
    return analogRead(SENSOR_PIN) * (5.0f / 1023.0f) * 100.0f;
}

void setup()
{
    Serial.begin(115200);
    pinMode(HEATER_PIN, OUTPUT);

    shark_pid_cfg_t cfg;
    shark_pid_cfg_default(&cfg);

    cfg.kp = 8.0f;
    cfg.ki = 0.6f;                      // đơn vị 1/giây — KHÔNG phụ thuộc PERIOD_MS
    cfg.kd = 12.0f;                     // đơn vị giây

    cfg.out_min = 0.0f;                 // sò chỉ nung, không làm lạnh
    cfg.out_max = 255.0f;
    cfg.i_min   = 0.0f;                 // tích phân cũng chỉ được dương
    cfg.i_max   = 255.0f;

    cfg.d_tau = 0.30f;                  // lọc D 300 ms: nhiệt độ đổi chậm, nhiễu thì nhanh
    cfg.deadband = 0.3f;                // trong ±0.3 °C thì thôi nhấp nhô

    // Biến tốc độ tích phân: còn cách hơn 25 °C thì tắt hẳn khâu I,
    // vào trong 5 °C mới mở hết. Chống windup lúc khởi động nguội.
    cfg.ci_a = 20.0f;
    cfg.ci_b = 5.0f;

    cfg.dt_nominal = PERIOD_MS / 1000.0f;

    cfg.flags = SHARK_PID_F_TRAPEZOID_I | SHARK_PID_F_CLAMP_I;

    pid.core().cfg = cfg;
    pid.reset();
}

void loop()
{
    static uint32_t last = 0;
    uint32_t now = millis();

    if (now - last >= PERIOD_MS) {      // trừ unsigned -> đúng cả khi millis() tràn
        last = now;

        float tempC = readTemperatureC();
        float u = pid.update(SETPOINT_C, tempC);   // dt tự đo bằng micros()

        analogWrite(HEATER_PIN, (int)(u + 0.5f));

        Serial.print(F("T=")); Serial.print(tempC, 2);
        Serial.print(F("  u=")); Serial.print(u, 1);
        Serial.print(F("  P=")); Serial.print(pid.pTerm(), 1);
        Serial.print(F("  I=")); Serial.print(pid.iTerm(), 1);
        Serial.print(F("  D=")); Serial.print(pid.dTerm(), 1);
        if (pid.isSaturated()) Serial.print(F("  [BAO HOA]"));
        Serial.println();
    }

    // CPU vẫn rảnh cho việc khác — không có delay() nào ở đây.
}
