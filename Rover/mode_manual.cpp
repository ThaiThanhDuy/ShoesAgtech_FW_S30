#include "Rover.h"

void ModeManual::_exit() {
  // clear lateral when exiting manual mode
  g2.motors.set_lateral(0);
}

void ModeManual::update() {
  float desired_steering, desired_throttle, desired_lateral;
  get_pilot_desired_steering_and_throttle(desired_steering, desired_throttle);
  get_pilot_desired_lateral(desired_lateral);

  // // --- Pitch Safety Warning Logic START ---
  // const float pitch_deg = degrees(rover.ahrs.get_pitch());

  // // Kiểm tra trạng thái góc Pitch dựa trên tham số cấu hình hệ thống
  // // g.safe_pitch_down: Ngưỡng chúi mũi | g.safe_pitch_up: Ngưỡng ngóc mũi
  // const bool is_pitch_bad = (pitch_deg < -fabsf(g.safe_pitch_down.get()) ||
  //                            pitch_deg > fabsf(g.safe_pitch_up.get()));

  // if (is_pitch_bad) {
  //   const uint32_t now_ms = AP_HAL::millis();
  //   static uint32_t last_warn_ms = 0;
  //   desired_throttle = 0.0f;
  //   if (now_ms - last_warn_ms > 1000) { // Tần suất cảnh báo 1Hz
  //     gcs().send_text(MAV_SEVERITY_CRITICAL,
  //                     "PITCH DANGER: %.2f deg | THROTTLE LOCKED",
  //                     (double)pitch_deg);
  //     last_warn_ms = now_ms;
  //   }
  // }
  // // --- Pitch Safety Warning Logic END ---
  // Duy Update - Thêm vào tính toán gia tốc
  // --- Pitch Safety Warning Logic START (Angle & Acceleration Predictive
  // Control) --- Khởi tạo các biến thời gian thực của hệ thống
  const uint32_t now_ms = AP_HAL::millis();
  static uint32_t last_warn_ms = 0U;

  // 1. Trích xuất dữ liệu độ nghiêng tĩnh và vận tốc góc trục Y (Pitch rate) từ
  // IMU
  const float pitch_deg = degrees(rover.ahrs.get_pitch());
  const Vector3f &gyro = rover.ahrs.get_gyro();
  const float current_pitch_rate_rads = gyro.y;

  // 2. Tính toán gia tốc góc Pitch thô (Raw Angular Acceleration) bằng sai phân
  // hữu hạn
  float raw_pitch_accel_degs2 = 0.0f;
  if (rover.G_Dt >
      0.0001f) { // Ngăn chặn triệt để lỗi chia cho 0 (Undefined Behavior)
    raw_pitch_accel_degs2 =
        degrees(current_pitch_rate_rads - _manual_last_pitch_rate_rads) /
        rover.G_Dt;
  }
  _manual_last_pitch_rate_rads =
      current_pitch_rate_rads; // Lưu cấu trúc cho chu kỳ kế tiếp

  // 3. Áp dụng bộ lọc thông thấp (Low-Pass Filter) tần số cắt ~4Hz để triệt
  // nhiễu rung động cơ Hobbywing X8/X6 Hằng số thời gian RC = 1/(2*pi*f_cut) =
  // 0.04s. Hệ số alpha = dt / (RC + dt)
  const float lpf_alpha =
      constrain_float(rover.G_Dt / (0.04f + rover.G_Dt), 0.05f, 1.0f);
  _manual_filtered_pitch_accel_degs2 =
      (lpf_alpha * raw_pitch_accel_degs2) +
      ((1.0f - lpf_alpha) * _manual_filtered_pitch_accel_degs2);

  // 4. Khởi tạo ngưỡng động học từ hệ thống tham số toàn cục của Rover
  // (GSCALAR)
  const float safe_pitch_down_limit = -fabsf(g.safe_pitch_down.get());
  const float safe_pitch_up_limit = fabsf(g.safe_pitch_up.get());
  const float safe_pitch_accel_limit = fabsf(rover.g.safe_pitch_accel.get());

  // 5. Kiểm tra trạng thái: Vi phạm góc tĩnh HOẶC Vi phạm xung quán tính động
  // lực học
  const bool is_angle_bad =
      (pitch_deg < safe_pitch_down_limit) || (pitch_deg > safe_pitch_up_limit);
  const bool is_inertia_bad =
      (fabsf(_manual_filtered_pitch_accel_degs2) > safe_pitch_accel_limit);
  const bool is_pitch_bad = is_angle_bad || is_inertia_bad;

  if (is_pitch_bad) {
    desired_throttle =
        0.0f; // Khóa cứng đầu ra công suất, triệt tiêu xung ESC lập tức

    if (now_ms - last_warn_ms > 1000U) { // Tần suất cảnh báo chuẩn hóa 1Hz
      gcs().send_text(
          MAV_SEVERITY_CRITICAL,
          "PITCH DANGER! Ang:%.1fdeg Acc:%.1fdeg/s2 | THROTTLE LOCKED",
          static_cast<double>(pitch_deg),
          static_cast<double>(_manual_filtered_pitch_accel_degs2));
      last_warn_ms = now_ms;
    }
  }
  // --- Pitch Safety Warning Logic END ---
  // apply manual steering expo
  desired_steering =
      4500.0f * input_expo(desired_steering / 4500.0f, g2.manual_steering_expo);

  // if vehicle is balance bot, calculate actual throttle required for balancing
  if (rover.is_balancebot()) {
    rover.balancebot_pitch_control(desired_throttle);
  }

  // walking robots support roll, pitch and walking_height
  float desired_roll, desired_pitch, desired_walking_height;
  get_pilot_desired_roll_and_pitch(desired_roll, desired_pitch);
  get_pilot_desired_walking_height(desired_walking_height);
  g2.motors.set_roll(desired_roll);
  g2.motors.set_pitch(desired_pitch);
  g2.motors.set_walking_height(desired_walking_height);

  // set sailboat sails
  g2.sailboat.set_pilot_desired_mainsail();

  // copy RC scaled inputs to outputs
  g2.motors.set_throttle(desired_throttle);
  g2.motors.set_steering(desired_steering,
                         (g2.manual_options & ManualOptions::SPEED_SCALING));
  g2.motors.set_lateral(desired_lateral);
}