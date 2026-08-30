/**
 * 01_Heater_Basic — Điều khiển nhiệt độ lò sấy (cơ cấu một chiều).
 *
 * Đặc điểm bài toán:
 *   - Sò công suất chỉ nung được, không làm lạnh -> ngõ ra [0, 255].
 *   - Cảm biến nhiệt có nhiễu -> cần lọc khâu D (N nhỏ = lọc mạnh).
 *   - Quán tính nhiệt lớn -> rất dễ windup khi khởi động nguội, nên kẹp
 *     tích phân bằng i_min/i_max cộng chống windup kiểu clamping.
 *
 * Mọi giá trị dưới đây điền thẳng được vào khối `PID Controller (2DOF)`
 * của Simulink — chạy `shark_pid_map2simulink` trong thư mục extras/Test_Shark_PID/ để
 * lấy bảng ánh xạ.
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

    cfg.kp = 8.0f;                      // ô P
    cfg.ki = 0.6f;                      // ô I, đơn vị 1/giây — KHÔNG phụ thuộc PERIOD_MS
    cfg.kd = 12.0f;                     // ô D, đơn vị giây

    cfg.b = 1.0f;                       // Setpoint weight (b)
    cfg.c = 0.0f;                       // Setpoint weight (c): D chỉ nhìn cảm biến

    cfg.out_min = 0.0f;                 // sò chỉ nung, không làm lạnh
    cfg.out_max = 255.0f;
    cfg.i_min   = 0.0f;                 // tích phân cũng chỉ được dương
    cfg.i_max   = 255.0f;

    // Filter coefficient N, đơn vị rad/giây. N = 3.33 tương đương hằng số
    // thời gian 300 ms: nhiệt độ đổi chậm còn nhiễu thì nhanh.
    cfg.n = 3.33f;

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
