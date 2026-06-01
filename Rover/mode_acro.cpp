#include "Rover.h"

// UPDATE 3 - safe góc pitch và giới hạn vọt lố speed setpoint (ramprate)
// void ModeAcro::update() {
//   // === 1. BIẾN STATIC VÀ KHỞI TẠO ===
//   static int8_t last_rail_enable = -1;
//   static float last_rail_speed = -1.0f;
//   static float last_rail_ramped_rate = -1.0f;
//   static float ramped_speed = 0.0f;
//   static bool last_armed = false;
//   static uint32_t last_rail_update_ms = 0;
//   static uint32_t last_log_ms = 0;
//   float actual_speed = 0.0f;
//   const uint32_t now_ms = AP_HAL::millis();
//   const float dt = rover.G_Dt;
//   float accel_step = 0.0f;
//   // Theo dõi trạng thái hệ thống
//   const bool currently_armed = rover.arming.is_armed();
//   const bool just_armed = (currently_armed && !last_armed);
//   const bool new_rail_session = (now_ms - last_rail_update_ms > 300);
//   last_rail_update_ms = now_ms;

//   // Biến cờ để kiểm soát hiển thị cảnh báo Pitch
//   static bool pitch_warning_sent = false;
//   // Thu thập parameters
//   const int8_t current_rail_enable = (int8_t)g.rail_enable.get();
//   const float current_rail_speed = g.rail_speed.get();
//   const float current_rail_ramped_rate = g.rail_ramped_rate.get();

//   // --- QUẢN LÝ TRẠNG THÁI CHUYỂN ĐỔI ---
//   bool state_changed =
//       (current_rail_enable != last_rail_enable) ||
//       (!is_equal(current_rail_speed, last_rail_speed)) ||
//       (!is_equal(current_rail_ramped_rate, last_rail_ramped_rate));

//   if (state_changed) {
//     if (current_rail_enable == 1) {
//       if (last_rail_enable != 1)
//         ramped_speed = 0.0f;
//       gcs().send_text(MAV_SEVERITY_INFO, "Rail: ACTIVE | Target: %.2f m/s",
//                       (double)current_rail_speed);
//     } else {
//       if (last_rail_enable == 1)
//         gcs().send_text(MAV_SEVERITY_INFO, "Rail: DEACTIVATE");
//       ramped_speed = 0.0f;
//     }
//     last_rail_enable = current_rail_enable;
//     last_rail_speed = current_rail_speed;
//     last_rail_ramped_rate = current_rail_ramped_rate;
//   }

//   // Lấy tốc độ thực tế từ EKF
//   const bool speed_available =
//   attitude_control.get_forward_speed(actual_speed);

//   // Reset logic khi mới Arm hoặc vào lại session
//   if (current_rail_enable == 1 && (just_armed || new_rail_session)) {
//     ramped_speed = speed_available ? actual_speed : 0.0f;
//   }
//   last_armed = currently_armed;

//   // === 2. LOGIC ĐIỀU KHIỂN CHÍNH (RAIL MODE) ===
//   if (current_rail_enable == 1) {
//     float target_v = current_rail_speed;

//     // --- STAGE 2.1: PITCH SAFETY (UP/DOWN) ---
//     if (g.rail_safe_pitch_en.get() == 1) {
//       const float current_pitch_deg = degrees(rover.ahrs.get_pitch());
//       const float limit_dn = -fabsf((float)g.rail_safe_pitch_down.get());
//       const float limit_up = fabsf((float)g.rail_safe_pitch_up.get());
//       const float percent_scale_safety = 0.70f; // 70%
//       if (current_pitch_deg < limit_dn || current_pitch_deg > limit_up) {
//         target_v = current_rail_speed * percent_scale_safety;

//         // Chỉ gửi tin nhắn nếu chưa gửi trước đó (One-time alert)
//         if (!pitch_warning_sent) {
//           gcs().send_text(MAV_SEVERITY_WARNING,
//                           "Rail: Pitch Safety ACTIVE (70%% Speed)");
//           pitch_warning_sent = true;
//         }
//       } else {
//         // Khi quay lại góc an toàn, reset cờ và thông báo (tùy chọn)
//         if (pitch_warning_sent) {
//           gcs().send_text(MAV_SEVERITY_INFO,
//                           "Rail: Pitch Safe - Resuming speed");
//           pitch_warning_sent = false;
//         }
//       }
//     }

//     // --- STAGE 2.2: ADAPTIVE RAMP LOGIC ---
//     if (speed_available) {
//       accel_step = current_rail_ramped_rate * dt;
//       // Giới hạn ramped_speed dẫn trước thực tế tối đa (m/s) để chống
//       I-Windup const float spd_lead = MAX(g.rail_speed_lead.get(), 0.1f);

//       const float max_lead = actual_speed + spd_lead;
//       const float min_lead = actual_speed - spd_lead;

//       if (ramped_speed < target_v) {
//         ramped_speed += accel_step;
//         if (ramped_speed > target_v)
//           ramped_speed = target_v;
//         if (ramped_speed > max_lead)
//           ramped_speed = max_lead; // Kìm hãm ramp nếu máy chưa bám kịp
//       } else if (ramped_speed > target_v) {
//         ramped_speed -= accel_step;
//         if (ramped_speed > target_v)
//           ramped_speed = target_v;
//         if (ramped_speed < min_lead)
//           ramped_speed = min_lead;
//       }
//       calc_throttle(ramped_speed, false);
//     } else {
//       // Trường hợp mất phản hồi tốc độ: Sử dụng giá trị phần trăm ga cố định
//       g2.motors.set_throttle(g.rail_percent * 0.01f);
//     }
//     // --- STAGE 2.3: STEERING PROCESS ---
//     float pilot_steering, pilot_throttle;
//     get_pilot_desired_steering_and_throttle(pilot_steering, pilot_throttle);
//     const float steer_dz = (float)g.rail_steer_dz.get();
//     float target_turn_rate_rad = 0.0f;

//     if (g.rail_auto_steer.get() == 1) {
//       // Thuật toán auto cua
//       if (pilot_steering > steer_dz) {
//         target_turn_rate_rad = radians(fabsf(g.rail_auto_turn_rate.get()));
//       } else if (pilot_steering < -steer_dz) {
//         target_turn_rate_rad = -radians(fabsf(g.rail_auto_turn_rate.get()));
//       }
//     } else {
//       if (fabsf(pilot_steering) > steer_dz) {
//         target_turn_rate_rad =
//             (pilot_steering / 4500.0f) * radians(g2.acro_turn_rate);
//       }
//     }

//     float steering_out = attitude_control.get_steering_out_rate(
//         target_turn_rate_rad, g2.motors.limit.steer_left,
//         g2.motors.limit.steer_right, dt);

//     set_steering(constrain_float(steering_out, -1.0f, 1.0f) * 4500.0f);
//     if (g.rail_log_enable.get() == 1) {
//       if (now_ms - last_log_ms > 2000) {
//         // Log gộp các thông số quan trọng để tiết kiệm băng thông MAVLink
//         if (g.rail_log_enable.get() == 1) {
//           if (now_ms - last_log_ms > 2000) {
//             gcs().send_text(MAV_SEVERITY_INFO, "T:%.1f R:%.2f A:%.2f
//             STR:%.3f",
//                             (double)target_v, (double)ramped_speed,
//                             (double)actual_speed,
//                             (double)target_turn_rate_rad);
//             last_log_ms = now_ms;
//           }
//         }
//         last_log_ms = now_ms;
//       }
//     }
//   } else {
//     // === 3. DEFAULT ARDUPILOT ACRO MODE ===
//     float speed, desired_steering;
//     if (!attitude_control.get_forward_speed(speed)) {
//       float desired_throttle;
//       get_pilot_desired_steering_and_throttle(desired_steering,
//                                               desired_throttle);
//       if (rover.is_balancebot())
//         rover.balancebot_pitch_control(desired_throttle);
//       g2.motors.set_throttle(desired_throttle);
//     } else {
//       float desired_speed;
//       get_pilot_desired_steering_and_speed(desired_steering, desired_speed);
//       calc_throttle(desired_speed, true);
//     }

//     float steering_out;
//     if (!is_zero(desired_steering))
//       g2.sailboat.clear_tack();

//     if (g2.sailboat.tacking()) {
//       steering_out = attitude_control.get_steering_out_heading(
//           g2.sailboat.get_tack_heading_rad(), g2.wp_nav.get_pivot_rate(),
//           g2.motors.limit.steer_left, g2.motors.limit.steer_right, dt);
//     } else {
//       const float target_turn_rate =
//           (desired_steering / 4500.0f) * radians(g2.acro_turn_rate);
//       steering_out = attitude_control.get_steering_out_rate(
//           target_turn_rate, g2.motors.limit.steer_left,
//           g2.motors.limit.steer_right, dt);
//     }
//     set_steering(steering_out * 4500.0f);
//   }
// }
// Update V4 - tối ưu hóa code và clean code
// void ModeAcro::update() {
//   // === 1. INITIALIZATION ===
//   const uint32_t now_ms = AP_HAL::millis();
//   const float dt = rover.G_Dt;
//   if (dt <= 0.0001f || dt > 0.1f) {
//     return;
//   }
//   const int8_t rail_en = static_cast<int8_t>(rover.g.rail_enable.get());

//   const bool currently_armed = rover.arming.is_armed();
//   const bool just_armed = (currently_armed && !_rail_last_armed);
//   const bool new_session = (now_ms - _rail_last_update_ms > 300U);

//   _rail_last_update_ms = now_ms;
//   _rail_last_armed = currently_armed;

//   // === 2. RAIL MODE LOGIC ===
//   if (rail_en == 1) {
//     // --- 2.1: State Change Handling ---
//     const float target_v_param = rover.g.rail_speed.get();
//     if (rail_en != _rail_last_enable ||
//         !is_equal(target_v_param, _rail_last_speed)) {
//       if (_rail_last_enable != 1) {
//         _rail_ramped_speed = 0.0f;
//       }
//       gcs().send_text(MAV_SEVERITY_INFO, "[RAIL] Rail:ACTIVE | Target: %.2f",
//                       static_cast<double>(target_v_param));
//       _rail_last_enable = rail_en;
//       _rail_last_speed = target_v_param;
//     }

//     float actual_speed = 0.0f;
//     const bool speed_available =
//         attitude_control.get_forward_speed(actual_speed);

//     // Reset Setpoint to actual speed when starting a new session or Arming
//     if (just_armed || new_session) {
//       _rail_ramped_speed = speed_available ? actual_speed : 0.0f;
//       _rail_pitch_safe_start_ms = 0U; // Reset recovery timer
//     }

//     // --- 2.2: Pitch Safety Logic (Speed Saturation Limit) ---
//     float target_v = target_v_param;
//     if (rover.g.rail_safe_pitch_en.get() == 1) {
//       // 2.2.1. Trích xuất dữ liệu độ nghiêng tĩnh và vận tốc góc trục Y
//       (Pitch
//       // rate) từ IMU
//       const float pitch_deg = degrees(rover.ahrs.get_pitch());
//       const Vector3f &gyro = rover.ahrs.get_gyro();
//       const float current_pitch_rate_rads = gyro.y;

//       // 2.2.2. Tính toán gia tốc góc Pitch thô (Raw Angular Acceleration)
//       bằng
//       // sai phân hữu hạn
//       float raw_pitch_accel_degs2 = 0.0f;
//       if (rover.G_Dt >
//           0.0001f) { // Ngăn chặn triệt để lỗi chia cho 0 (Undefined
//           Behavior)
//         raw_pitch_accel_degs2 =
//             degrees(current_pitch_rate_rads - _rail_last_pitch_rate_rads) /
//             rover.G_Dt;
//       }
//       _rail_last_pitch_rate_rads =
//           current_pitch_rate_rads; // Lưu cấu trúc cho chu kỳ kế tiếp

//       // 2.2.3. Áp dụng bộ lọc thông thấp (Low-Pass Filter) tần số cắt ~4Hz
//       để
//       // triệt nhiễu động cơ Hobbywing Hằng số thời gian RC = 1/(2*pi*f_cut)
//       =
//       // 0.04s. Hệ số alpha = dt / (RC + dt)
//       const float lpf_alpha =
//           constrain_float(rover.G_Dt / (0.04f + rover.G_Dt), 0.05f, 1.0f);
//       _rail_filtered_pitch_accel_degs2 =
//           (lpf_alpha * raw_pitch_accel_degs2) +
//           ((1.0f - lpf_alpha) * _rail_filtered_pitch_accel_degs2);

//       // 2.2.4. Khởi tạo ngưỡng động học từ hệ thống tham số toàn cục của
//       Rover const float safe_pitch_down_limit =
//       -fabsf(rover.g.safe_pitch_down.get()); const float safe_pitch_up_limit
//       = fabsf(rover.g.safe_pitch_up.get()); const float
//       safe_pitch_accel_limit =
//           fabsf(rover.g.safe_pitch_accel.get()); // Tham số GSCALAR mới tích
//           hợp

//       // 2.2.5. Kiểm tra trạng thái: Vi phạm góc tĩnh HOẶC Vi phạm xung quán
//       // tính động lực học
//       const bool is_angle_bad = (pitch_deg < safe_pitch_down_limit) ||
//                                 (pitch_deg > safe_pitch_up_limit);
//       const bool is_inertia_bad =
//           (fabsf(_rail_filtered_pitch_accel_degs2) > safe_pitch_accel_limit);
//       const bool is_pitch_bad = is_angle_bad || is_inertia_bad;

//       // Khai báo tỷ lệ scale hỗ trợ dải rộng từ 0% đến 100%
//       const float pitch_scale_param = rover.g.rail_pitch_scale.get() * 0.01f;
//       const float pitch_scale = constrain_float(pitch_scale_param,
//       0.0f, 1.0f);

//       if (is_pitch_bad) {
//         // Áp trần tốc độ đích dựa trên tham số đầu vào target_v_param để
//         triệt
//         // tiêu động năng lật
//         const float max_allowed_speed = target_v_param * pitch_scale;
//         if (target_v > max_allowed_speed) {
//           target_v = max_allowed_speed;
//         }
//         _rail_pitch_safe_start_ms =
//             0U; // Reset timer phục hồi khi hệ thống vẫn đang vi phạm

//         if (!_rail_pitch_warning_sent) {
//           gcs().send_text(MAV_SEVERITY_CRITICAL,
//                           "[RAIL] PITCH DANGER! Ang:%.1fdeg Acc:%.1fdeg/s2 ->
//                           " "LIMIT TO %.2f m/s",
//                           static_cast<double>(pitch_deg),
//                           static_cast<double>(_rail_filtered_pitch_accel_degs2),
//                           static_cast<double>(max_allowed_speed));
//           _rail_pitch_warning_sent = true;
//         }
//       } else if (_rail_pitch_warning_sent) {
//         if (_rail_pitch_safe_start_ms == 0U) {
//           _rail_pitch_safe_start_ms = now_ms;
//         }

//         const uint32_t recovery_delay_ms =
//             static_cast<uint32_t>(MAX(rover.g.rail_pitch_delay.get(), 0.0f));

//         if (now_ms - _rail_pitch_safe_start_ms >= recovery_delay_ms) {
//           gcs().send_text(
//               MAV_SEVERITY_WARNING,
//               "[RAIL] Pitch & Inertia Safe - Resuming normal speed");
//           _rail_pitch_warning_sent = false;
//           _rail_pitch_safe_start_ms = 0U;
//         } else {
//           // Duy trì giới hạn tốc độ bảo vệ trong suốt chu kỳ thời gian trễ
//           phục
//           // hồi (Recovery Window)
//           const float max_allowed_speed = target_v_param * pitch_scale;
//           if (target_v > max_allowed_speed) {
//             target_v = max_allowed_speed;
//           }
//         }
//       }
//     } else {
//       // Reset bộ lọc lưu vết nếu tính năng an toàn bị tắt (Disable) trên GCS
//       để
//       // tránh lỗi tích lũy sai phân
//       _rail_last_pitch_rate_rads = 0.0f;
//       _rail_filtered_pitch_accel_degs2 = 0.0f;
//     }
//     // --- 2.3: Adaptive Ramp Logic ---
//     if (speed_available) {
//       const float accel_step = rover.g.rail_ramped_rate.get() * dt;
//       const float spd_lead = MAX(rover.g.rail_speed_lead.get(), 0.1f);

//       // Prevent I-Windup by constraining ramped speed around actual speed
//       _rail_ramped_speed = constrain_float(
//           _rail_ramped_speed, actual_speed - spd_lead, actual_speed +
//           spd_lead);

//       // Smooth profile generator to target_v
//       if (_rail_ramped_speed < target_v) {
//         _rail_ramped_speed = MIN(target_v, _rail_ramped_speed + accel_step);
//       } else if (_rail_ramped_speed > target_v) {
//         _rail_ramped_speed = MAX(target_v, _rail_ramped_speed - accel_step);
//       }
//       calc_throttle(_rail_ramped_speed, false);
//     } else {
//       // Failsafe: Standard raw duty cycle when EKF feedback is lost
//       g2.motors.set_throttle(rover.g.rail_percent * 0.01f);
//     }

//     // --- 2.4: Steering Process ---
//     float p_steer, p_throt;
//     get_pilot_desired_steering_and_throttle(p_steer, p_throt);
//     float target_turn_rad = 0.0f;
//     const float steer_dz = static_cast<float>(rover.g.rail_steer_dz.get());

//     if (rover.g.rail_auto_steer.get() == 1 && fabsf(p_steer) > steer_dz) {
//       target_turn_rad = (p_steer > 0.0f)
//                             ? radians(rover.g.rail_auto_turn_rate.get())
//                             : -radians(rover.g.rail_auto_turn_rate.get());
//     } else if (fabsf(p_steer) > steer_dz) {
//       target_turn_rad = (p_steer / 4500.0f) * radians(g2.acro_turn_rate);
//     }

//     float steer_out = attitude_control.get_steering_out_rate(
//         target_turn_rad, g2.motors.limit.steer_left,
//         g2.motors.limit.steer_right, dt);
//     set_steering(constrain_float(steer_out, -1.0f, 1.0f) * 4500.0f);

//     // --- 2.5: Clean Telemetry Logging (2Hz) ---
//     if (rover.g.rail_log_enable.get() == 1 &&
//         (now_ms - _rail_last_log_ms > 2000U)) {
//       gcs().send_text(MAV_SEVERITY_INFO, "[RAIL] T:%.1f R:%.2f A:%.2f
//       STR:%.3f",
//                       static_cast<double>(target_v),
//                       static_cast<double>(_rail_ramped_speed),
//                       static_cast<double>(actual_speed),
//                       static_cast<double>(target_turn_rad));
//       _rail_last_log_ms = now_ms;
//     }
//   } else {
//     // === 3. DEFAULT ARDUPILOT ACRO MODE (EXIT RAIL) ===
//     if (_rail_last_enable == 1) {
//       gcs().send_text(MAV_SEVERITY_INFO, "[RAIL] Rail:DEACTIVATE");
//       _rail_last_enable = 0;
//       _rail_pitch_warning_sent = false;
//       _rail_pitch_safe_start_ms = 0U;
//     }

//     float speed, desired_steering;
//     if (!attitude_control.get_forward_speed(speed)) {
//       float desired_throttle;
//       get_pilot_desired_steering_and_throttle(desired_steering,
//                                               desired_throttle);
//       if (rover.is_balancebot()) {
//         rover.balancebot_pitch_control(desired_throttle);
//       }
//       g2.motors.set_throttle(desired_throttle);
//     } else {
//       float desired_speed;
//       get_pilot_desired_steering_and_speed(desired_steering, desired_speed);
//       calc_throttle(desired_speed, true);
//     }

//     if (!is_zero(desired_steering)) {
//       g2.sailboat.clear_tack();
//     }

//     float steering_out;
//     if (g2.sailboat.tacking()) {
//       steering_out = attitude_control.get_steering_out_heading(
//           g2.sailboat.get_tack_heading_rad(), g2.wp_nav.get_pivot_rate(),
//           g2.motors.limit.steer_left, g2.motors.limit.steer_right, dt);
//     } else {
//       const float target_turn_rate =
//           (desired_steering / 4500.0f) * radians(g2.acro_turn_rate);
//       steering_out = attitude_control.get_steering_out_rate(
//           target_turn_rate, g2.motors.limit.steer_left,
//           g2.motors.limit.steer_right, dt);
//     }
//     set_steering(steering_out * 4500.0f);
//   }
// }

// Bộ lọc thông thấp tĩnh cho gia tốc góc trục Y (Pitch rate) - f_cut = 4.0 Hz
static LowPassFilterFloat pitch_rate_derivative_lpf(4.0f);

void ModeAcro::update() {
  // === 1. INITIALIZATION & INPUT SANITIZATION ===
  const uint32_t now_ms = AP_HAL::millis();
  const float dt = rover.G_Dt;

  // Ngăn chặn lỗi chia cho 0 và chu kỳ bất thường hệ thống (NASA Software
  // Safety Guidelines)
  if (dt <= 0.0001f || dt > 0.1f) {
    return;
  }

  const int8_t rail_en = static_cast<int8_t>(rover.g.rail_enable.get());
  const bool currently_armed = rover.arming.is_armed();
  const bool just_armed = (currently_armed && !_rail_last_armed);
  const bool new_session = (now_ms - _rail_last_update_ms > 300U);

  _rail_last_update_ms = now_ms;
  _rail_last_armed = currently_armed;

  // === 2. RAIL MODE EXECUTION PATH ===
  if (rail_en == 1) {
    // --- 2.1: State and Parameter Sync ---
    const float target_v_param = rover.g.rail_speed.get();
    if (rail_en != _rail_last_enable ||
        !is_equal(target_v_param, _rail_last_speed)) {
      if (_rail_last_enable != 1) {
        _rail_ramped_speed = 0.0f;
      }
      gcs().send_text(MAV_SEVERITY_INFO,
                      "[RAIL] Active | Target Velocity: %.2f m/s",
                      static_cast<double>(target_v_param));
      _rail_last_enable = rail_en;
      _rail_last_speed = target_v_param;

      // Thiết lập lại máy trạng thái về giai đoạn đề ba vận tốc
      _rail_sub_mode = RailSubMode::VELOCITY_RAMP;
      _rail_velocity_target_achieved = false;
      _rail_velocity_reached_ms = 0U;
    }

    float actual_speed = 0.0f;
    const bool speed_available =
        attitude_control.get_forward_speed(actual_speed);

    // Khởi tạo lại trạng thái hệ thống khi có tín hiệu Arm hoặc Session mới
    if (just_armed || new_session) {
      _rail_ramped_speed = speed_available ? actual_speed : 0.0f;
      _rail_pitch_safe_start_ms = 0U;
      _rail_sub_mode = RailSubMode::VELOCITY_RAMP;
      _rail_velocity_target_achieved = false;
      _rail_velocity_reached_ms = 0U;
    }

    // --- 2.2: Advanced Pitch Safety Logic (Speed Saturation Limit) ---
    float target_v = target_v_param;
    if (rover.g.rail_safe_pitch_en.get() == 1) {
      const float pitch_deg = degrees(rover.ahrs.get_pitch());
      const Vector3f &gyro = rover.ahrs.get_gyro();
      const float current_pitch_rate_rads = gyro.y;

      const float raw_pitch_accel_degs2 =
          degrees(current_pitch_rate_rads - _rail_last_pitch_rate_rads) / dt;
      _rail_last_pitch_rate_rads = current_pitch_rate_rads;

      // Lọc dữ liệu chống nhiễu dải tần cơ khí cao từ cụm động cơ Hobbywing
      // X6/X8
      _rail_filtered_pitch_accel_degs2 =
          pitch_rate_derivative_lpf.apply(raw_pitch_accel_degs2, dt);

      const float safe_pitch_down_limit = -fabsf(rover.g.safe_pitch_down.get());
      const float safe_pitch_up_limit = fabsf(rover.g.safe_pitch_up.get());
      const float safe_pitch_accel_limit =
          fabsf(rover.g.safe_pitch_accel.get());

      const bool is_angle_bad = (pitch_deg < safe_pitch_down_limit) ||
                                (pitch_deg > safe_pitch_up_limit);
      const bool is_inertia_bad =
          (fabsf(_rail_filtered_pitch_accel_degs2) > safe_pitch_accel_limit);
      const bool is_pitch_bad = is_angle_bad || is_inertia_bad;

      const float pitch_scale_param = rover.g.rail_pitch_scale.get() * 0.01f;
      const float pitch_scale = constrain_float(pitch_scale_param, 0.0f, 1.0f);

      if (is_pitch_bad) {
        const float max_allowed_speed = target_v_param * pitch_scale;
        if (target_v > max_allowed_speed) {
          target_v = max_allowed_speed;
        }
        _rail_pitch_safe_start_ms = 0U;

        // Nếu mất an toàn cơ khí, bắt buộc nhảy tiến trình về Closed-loop vận
        // tốc để ghìm momen lật
        if (_rail_sub_mode != RailSubMode::VELOCITY_RAMP) {
          _rail_sub_mode = RailSubMode::VELOCITY_RAMP;
          _rail_velocity_target_achieved = false;
          _rail_velocity_reached_ms = 0U;
          gcs().send_text(
              MAV_SEVERITY_WARNING,
              "[RAIL] Pitch Danger! Forced back to Velocity Control");
        }

        if (!_rail_pitch_warning_sent) {
          gcs().send_text(
              MAV_SEVERITY_CRITICAL,
              "[RAIL] PITCH LIMIT ACTIVE! Ang:%.1f Acc:%.1f",
              static_cast<double>(pitch_deg),
              static_cast<double>(_rail_filtered_pitch_accel_degs2));
          _rail_pitch_warning_sent = true;
        }
      } else if (_rail_pitch_warning_sent) {
        if (_rail_pitch_safe_start_ms == 0U) {
          _rail_pitch_safe_start_ms = now_ms;
        }

        const uint32_t recovery_delay_ms =
            static_cast<uint32_t>(MAX(rover.g.rail_pitch_delay.get(), 0.0f));

        if (now_ms - _rail_pitch_safe_start_ms >= recovery_delay_ms) {
          gcs().send_text(MAV_SEVERITY_WARNING,
                          "[RAIL] Pitch Safe - Recovery Complete");
          _rail_pitch_warning_sent = false;
          _rail_pitch_safe_start_ms = 0U;
        } else {
          const float max_allowed_speed = target_v_param * pitch_scale;
          if (target_v > max_allowed_speed) {
            target_v = max_allowed_speed;
          }
        }
      }
    } else {
      _rail_last_pitch_rate_rads = 0.0f;
      _rail_filtered_pitch_accel_degs2 = 0.0f;
      pitch_rate_derivative_lpf.reset();
    }

    // --- 2.3: Hybrid Dual Closed-Loop Control Logic (If-Else Pattern) ---
    // ---
    if (speed_available) {
      const uint32_t target_duration_ms =
          static_cast<uint32_t>(MAX(1.0, 0.0f) * 1000.0f);

      // Đọc dữ liệu từ tầng Servo [0, 1000] và chuẩn hóa về dải hiển thị GCS
      // [0.0, 100.0]%
      const int16_t raw_srv_throttle =
          SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
      const float current_srv_throttle_percent =
          static_cast<float>(raw_srv_throttle) * 0.1f;

      if (_rail_sub_mode == RailSubMode::VELOCITY_RAMP) {
        const float accel_step = rover.g.rail_ramped_rate.get() * dt;
        const float spd_lead = MAX(rover.g.rail_speed_lead.get(), 0.1f);

        _rail_ramped_speed =
            constrain_float(_rail_ramped_speed, actual_speed - spd_lead,
                            actual_speed + spd_lead);

        if (_rail_ramped_speed < target_v) {
          _rail_ramped_speed = MIN(target_v, _rail_ramped_speed + accel_step);
        } else if (_rail_ramped_speed > target_v) {
          _rail_ramped_speed = MAX(target_v, _rail_ramped_speed - accel_step);
        }

        // Thực thi luồng điều khiển gốc của ArduPilot
        calc_throttle(_rail_ramped_speed, false);

        if (fabsf(actual_speed - target_v) <= 0.15f) {
          if (!_rail_velocity_target_achieved) {
            _rail_velocity_target_achieved = true;
            _rail_velocity_reached_ms = now_ms;
          } else if (now_ms - _rail_velocity_reached_ms >= target_duration_ms) {
            _rail_sub_mode = RailSubMode::THROTTLE_FOLLOW;
            gcs().send_text(MAV_SEVERITY_INFO,
                            "[RAIL] Transition to Actuator Closed-Loop");
          }
        } else {
          _rail_velocity_target_achieved = false;
          _rail_velocity_reached_ms = 0U;
        }
      } else if (_rail_sub_mode == RailSubMode::THROTTLE_FOLLOW) {
        const float target_throttle_percent =
            constrain_float(rover.g.rail_percent.get(), 0.0f, 100.0f);
        const float throttle_error =
            target_throttle_percent - current_srv_throttle_percent;

        // Hệ số tăng chỉnh tinh mịn dải gán tốc độ ảo
        const float k_throttle_loop_p = 0.02f;

        // Sử dụng chính biến _rail_ramped_speed làm tích phân vận tốc ảo để
        // điều khiển gián tiếp qua calc_throttle
        _rail_ramped_speed += (throttle_error * k_throttle_loop_p * dt);

        // Khống chế dải vận tốc ảo an toàn (Giới hạn tối đa bằng vận tốc cài
        // đặt ban đầu)
        _rail_ramped_speed =
            constrain_float(_rail_ramped_speed, 0.0f, target_v_param);

        // Gọi hàm calc_throttle để đồng bộ hóa luồng dữ liệu Mixer của
        // ArduPilot, ngăn chặn lỗi đè biến về 0
        calc_throttle(_rail_ramped_speed, false);

        // DEBUG LOG SYSTEM (Tần số quét 2Hz)
        static uint32_t last_follow_debug_ms = 0U;
        if (now_ms - last_follow_debug_ms >= 500U) {
          gcs().send_text(MAV_SEVERITY_DEBUG,
                          "[RAIL_FLW] Tgt:%.1f%% | Act:%.1f%% | Err:%.1f%% | "
                          "PseudoV:%.2f m/s",
                          static_cast<double>(target_throttle_percent),
                          static_cast<double>(current_srv_throttle_percent),
                          static_cast<double>(throttle_error),
                          static_cast<double>(_rail_ramped_speed));
          last_follow_debug_ms = now_ms;
        }
      }
    } else {
      // FAIL-SAFE PATTERN: Khi mất luồng EKF, đưa hệ thống về vòng kín vận tốc
      // an toàn hạ ga tuyến tính
      _rail_sub_mode = RailSubMode::VELOCITY_RAMP;
      _rail_velocity_target_achieved = false;
      _rail_velocity_reached_ms = 0U;

      const float deccel_step = rover.g.rail_ramped_rate.get() * dt;
      _rail_ramped_speed = MAX(0.0f, _rail_ramped_speed - deccel_step);

      const float safe_throttle_ratio =
          (_rail_ramped_speed / MAX(target_v_param, 0.1f));
      const float fallback_throttle =
          (rover.g.rail_percent.get() * 0.01f) * safe_throttle_ratio;

      g2.motors.set_throttle(constrain_float(fallback_throttle, 0.0f, 1.0f));
    }

    // --- 2.4: Steering Processing ---
    float p_steer, p_throt;
    get_pilot_desired_steering_and_throttle(p_steer, p_throt);
    float target_turn_rad = 0.0f;
    const float steer_dz = static_cast<float>(rover.g.rail_steer_dz.get());

    if (rover.g.rail_auto_steer.get() == 1 && fabsf(p_steer) > steer_dz) {
      target_turn_rad = (p_steer > 0.0f)
                            ? radians(rover.g.rail_auto_turn_rate.get())
                            : -radians(rover.g.rail_auto_turn_rate.get());
    } else if (fabsf(p_steer) > steer_dz) {
      target_turn_rad = (p_steer / 4500.0f) * radians(g2.acro_turn_rate);
    }

    const float steer_out = attitude_control.get_steering_out_rate(
        target_turn_rad, g2.motors.limit.steer_left,
        g2.motors.limit.steer_right, dt);
    set_steering(constrain_float(steer_out, -1.0f, 1.0f) * 4500.0f);

    // --- 2.5: Deterministic Telemetry Logging (2Hz) ---
    if (rover.g.rail_log_enable.get() == 1 &&
        (now_ms - _rail_last_log_ms > 2000U)) {
      gcs().send_text(MAV_SEVERITY_INFO, "[RAIL] LoopStage:%u V_Act:%.2f",
                      static_cast<uint8_t>(_rail_sub_mode),
                      static_cast<double>(actual_speed));
      _rail_last_log_ms = now_ms;
    }
  } else {
    // === 3. DEFAULT ARDUPILOT ACRO MODE (EXIT PATH) ===
    if (_rail_last_enable == 1) {
      gcs().send_text(MAV_SEVERITY_INFO, "[RAIL] Rail:DEACTIVATE");
      _rail_last_enable = 0;
      _rail_pitch_warning_sent = false;
      _rail_pitch_safe_start_ms = 0U;
      _rail_sub_mode = RailSubMode::VELOCITY_RAMP;
      _rail_velocity_target_achieved = false;
      _rail_velocity_reached_ms = 0U;
      pitch_rate_derivative_lpf.reset();
    }

    float speed, desired_steering;
    if (!attitude_control.get_forward_speed(speed)) {
      float desired_throttle;
      get_pilot_desired_steering_and_throttle(desired_steering,
                                              desired_throttle);
      if (rover.is_balancebot()) {
        rover.balancebot_pitch_control(desired_throttle);
      }
      g2.motors.set_throttle(desired_throttle);
    } else {
      float desired_speed;
      get_pilot_desired_steering_and_speed(desired_steering, desired_speed);
      calc_throttle(desired_speed, true);
    }

    if (!is_zero(desired_steering)) {
      g2.sailboat.clear_tack();
    }

    float steering_out;
    if (g2.sailboat.tacking()) {
      steering_out = attitude_control.get_steering_out_heading(
          g2.sailboat.get_tack_heading_rad(), g2.wp_nav.get_pivot_rate(),
          g2.motors.limit.steer_left, g2.motors.limit.steer_right, dt);
    } else {
      const float target_turn_rate =
          (desired_steering / 4500.0f) * radians(g2.acro_turn_rate);
      steering_out = attitude_control.get_steering_out_rate(
          target_turn_rate, g2.motors.limit.steer_left,
          g2.motors.limit.steer_right, dt);
    }
    set_steering(steering_out * 4500.0f);
  }
}
bool ModeAcro::requires_velocity() const {
  return !g2.motors.have_skid_steering();
}

// sailboats in acro mode support user manually initiating tacking from
// transmitter
void ModeAcro::handle_tack_request() { g2.sailboat.handle_tack_request_acro(); }