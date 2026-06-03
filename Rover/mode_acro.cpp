#include "Rover.h"

// Update V4 - tối ưu hóa code và clean code
// void ModeAcro::update() {
//   // === 1. INITIALIZATION & INPUT VALIDATION ===
//   const uint32_t now_ms = AP_HAL::millis();
//   const float dt = rover.G_Dt;

//   // Loại bỏ hoàn toàn hành vi không xác định (Undefined Behavior) từ lỗi
//   chia
//   // cho 0 hoặc sai số chu kỳ
//   if ((dt < 0.0001f) || (dt > 0.1f)) {
//     return;
//   }

//   const int8_t rail_en = static_cast<int8_t>(rover.g.rail_enable.get());
//   const bool currently_armed = rover.arming.is_armed();
//   const bool just_armed = (currently_armed && !rail_state_.last_armed);
//   const bool new_session = ((now_ms - rail_state_.last_update_ms) > 300U);

//   rail_state_.last_update_ms = now_ms;
//   rail_state_.last_armed = currently_armed;

//   // === 2. SYSTEM STATE MACHINE ===
//   if (rail_en == 1) {
//     // --- 2.1: State Change Handling ---
//     const float target_v_param = rover.g.rail_speed.get();
//     if ((rail_en != rail_state_.last_enable) ||
//         !is_equal(target_v_param, rail_state_.last_speed)) {
//       if (rail_state_.last_enable != 1) {
//         rail_state_.ramped_speed = 0.0f;
//       }
//       gcs().send_text(MAV_SEVERITY_INFO, "[RAIL] Rail:ACTIVE | Target: %.2f",
//                       static_cast<double>(target_v_param));
//       rail_state_.last_enable = rail_en;
//       rail_state_.last_speed = target_v_param;
//     }

//     float actual_speed = 0.0f;
//     const bool speed_available =
//         attitude_control.get_forward_speed(actual_speed);

//     // Đồng bộ Setpoint động học tại thời điểm khởi tạo chu trình mới hoặc
//     Arm
//     // máy
//     if (just_armed || new_session) {
//       rail_state_.ramped_speed = speed_available ? actual_speed : 0.0f;
//       rail_state_.pitch_safe_start_ms = 0U;
//     }

//     // --- 2.2: Pitch Safety Logic (Speed Saturation Limit) ---
//     float target_v = target_v_param;
//     if (rover.g.rail_safe_pitch_en.get() == 1) {
//       // 2.2.1. Trích xuất dữ liệu độ nghiêng tĩnh và vận tốc góc trục Y từ
//       IMU const float pitch_deg = degrees(rover.ahrs.get_pitch()); const
//       Vector3f &gyro = rover.ahrs.get_gyro(); const float
//       current_pitch_rate_deg_s = degrees(gyro.y);

//       // 2.2.2. Tính toán gia tốc góc Pitch thô (Raw Angular Acceleration)
//       const float raw_pitch_accel_deg_s2 =
//           (current_pitch_rate_deg_s - rail_state_.last_pitch_rate_deg_s) /
//           dt;
//       rail_state_.last_pitch_rate_deg_s = current_pitch_rate_deg_s;

//       // 2.2.3. Bộ lọc thông thấp (Low-Pass Filter) tần số cắt ~4Hz triệt
//       tiêu
//       // nhiễu xung cơ học 6 động cơ X8
//       const float lpf_alpha = constrain_float(dt / (0.04f + dt),
//       0.05f, 1.0f); rail_state_.filtered_pitch_accel_deg_s2 =
//           (lpf_alpha * raw_pitch_accel_deg_s2) +
//           ((1.0f - lpf_alpha) * rail_state_.filtered_pitch_accel_deg_s2);

//       // 2.2.4. Khởi tạo ngưỡng giới hạn động học hệ thống từ tham số toàn
//       cục
//       // GCS
//       const float safe_pitch_down_limit =
//       -fabsf(rover.g.safe_pitch_down.get()); const float safe_pitch_up_limit
//       = fabsf(rover.g.safe_pitch_up.get()); const float
//       safe_pitch_accel_limit =
//           fabsf(rover.g.safe_pitch_accel.get());

//       // 2.2.5. Kiểm tra điều kiện vi phạm: Góc nghiêng tĩnh HOẶC Xung quán
//       tính
//       // động lực học
//       const bool is_angle_bad = (pitch_deg < safe_pitch_down_limit) ||
//                                 (pitch_deg > safe_pitch_up_limit);
//       const bool is_inertia_bad =
//           (fabsf(rail_state_.filtered_pitch_accel_deg_s2) >
//            safe_pitch_accel_limit);
//       const bool is_pitch_bad = (is_angle_bad || is_inertia_bad);

//       const float pitch_scale_param = rover.g.rail_pitch_scale.get() * 0.01f;
//       const float pitch_scale = constrain_float(pitch_scale_param,
//       0.0f, 1.0f);

//       if (is_pitch_bad) {
//         const float max_allowed_speed = target_v_param * pitch_scale;
//         if (target_v > max_allowed_speed) {
//           target_v = max_allowed_speed;
//         }
//         rail_state_.pitch_safe_start_ms =
//             0U; // Khóa bộ đếm thời gian thực khi vẫn duy trì vi phạm

//         if (!rail_state_.pitch_warning_sent) {
//           gcs().send_text(
//               MAV_SEVERITY_CRITICAL,
//               "[RAIL] PITCH DANGER! Ang:%.1fdeg Acc:%.1fdeg/s2 -> LIMIT TO "
//               "%.2f m/s",
//               static_cast<double>(pitch_deg),
//               static_cast<double>(rail_state_.filtered_pitch_accel_deg_s2),
//               static_cast<double>(max_allowed_speed));
//           rail_state_.pitch_warning_sent = true;
//         }
//       } else if (rail_state_.pitch_warning_sent) {
//         if (rail_state_.pitch_safe_start_ms == 0U) {
//           rail_state_.pitch_safe_start_ms = now_ms;
//         }

//         const uint32_t recovery_delay_ms =
//             static_cast<uint32_t>(MAX(rover.g.rail_pitch_delay.get(), 0.0f));

//         if ((now_ms - rail_state_.pitch_safe_start_ms) >= recovery_delay_ms)
//         {
//           gcs().send_text(
//               MAV_SEVERITY_WARNING,
//               "[RAIL] Pitch & Inertia Safe - Resuming normal speed");
//           rail_state_.pitch_warning_sent = false;
//           rail_state_.pitch_safe_start_ms = 0U;
//         } else {
//           const float max_allowed_speed = target_v_param * pitch_scale;
//           if (target_v > max_allowed_speed) {
//             target_v = max_allowed_speed;
//           }
//         }
//       }
//     } else {
//       rail_state_.last_pitch_rate_deg_s = 0.0f;
//       rail_state_.filtered_pitch_accel_deg_s2 = 0.0f;
//     }

//     // --- 2.3: Adaptive Ramp Logic & Anti-Windup ---
//     if (speed_available) {
//       const float accel_step = rover.g.rail_ramped_rate.get() * dt;
//       const float spd_lead = MAX(rover.g.rail_speed_lead.get(), 0.1f);

//       // Ngăn chặn tích lũy sai số tích phân (Anti I-Windup) bằng cách giới
//       hạn
//       // biên độ quanh EKF
//       rail_state_.ramped_speed =
//           constrain_float(rail_state_.ramped_speed, actual_speed - spd_lead,
//                           actual_speed + spd_lead);

//       // Cấu hình tăng tốc mượt đến giá trị target_v
//       if (rail_state_.ramped_speed < target_v) {
//         rail_state_.ramped_speed =
//             MIN(target_v, rail_state_.ramped_speed + accel_step);
//       } else if (rail_state_.ramped_speed > target_v) {
//         rail_state_.ramped_speed =
//             MAX(target_v, rail_state_.ramped_speed - accel_step);
//       }
//       calc_throttle(rail_state_.ramped_speed, false);
//     } else {
//       // FAILSAFE: Ngắt toàn bộ công suất động cơ
//       g2.motors.set_throttle(0.0f);
//       set_steering(0.0f);
//       gcs().send_text(
//           MAV_SEVERITY_EMERGENCY,
//           "[RAIL] EMERGENCY: EKF Lost! Differential Thrust Cut Off");
//       return;
//     }

//     // --- 2.4: Steering Process (Differential Thrust Calculation) ---
//     float p_steer = 0.0f;
//     float p_throt = 0.0f;
//     get_pilot_desired_steering_and_throttle(p_steer, p_throt);
//     float target_turn_rad = 0.0f;
//     const float steer_dz = static_cast<float>(rover.g.rail_steer_dz.get());

//     if ((rover.g.rail_auto_steer.get() == 1) && (fabsf(p_steer) > steer_dz))
//     {
//       target_turn_rad = (p_steer > 0.0f)
//                             ? radians(rover.g.rail_auto_turn_rate.get())
//                             : -radians(rover.g.rail_auto_turn_rate.get());
//     } else if (fabsf(p_steer) > steer_dz) {
//       target_turn_rad = (p_steer / 4500.0f) * radians(g2.acro_turn_rate);
//     }

//     const float steer_out = attitude_control.get_steering_out_rate(
//         target_turn_rad, g2.motors.limit.steer_left,
//         g2.motors.limit.steer_right, dt);
//     set_steering(constrain_float(steer_out, -1.0f, 1.0f) * 4500.0f);

//     // --- 2.5: Clean Telemetry Logging (Hạ tần suất truyền dẫn hạ tầng không
//     // dây xuống 0.5Hz) ---
//     if ((rover.g.rail_log_enable.get() == 1) &&
//         ((now_ms - rail_state_.last_log_ms) > 2000U)) {
//       gcs().send_text(MAV_SEVERITY_INFO, "[RAIL] T:%.1f R:%.2f A:%.2f
//       STR:%.3f",
//                       static_cast<double>(target_v),
//                       static_cast<double>(rail_state_.ramped_speed),
//                       static_cast<double>(actual_speed),
//                       static_cast<double>(target_turn_rad));
//       rail_state_.last_log_ms = now_ms;
//     }
//   } else {
//     // === 3. DEFAULT ARDUPILOT ACRO MODE (EXIT RAIL SYSTEM) ===
//     if (rail_state_.last_enable == 1) {
//       gcs().send_text(MAV_SEVERITY_INFO, "[RAIL] Rail:DEACTIVATE");
//       rail_state_.last_enable = 0;
//       rail_state_.pitch_warning_sent = false;
//       rail_state_.pitch_safe_start_ms = 0U;
//     }

//     float speed = 0.0f;
//     float desired_steering = 0.0f;
//     if (!attitude_control.get_forward_speed(speed)) {
//       float desired_throttle = 0.0f;
//       get_pilot_desired_steering_and_throttle(desired_steering,
//                                               desired_throttle);
//       if (rover.is_balancebot()) {
//         rover.balancebot_pitch_control(desired_throttle);
//       }
//       g2.motors.set_throttle(desired_throttle);
//     } else {
//       float desired_speed = 0.0f;
//       get_pilot_desired_steering_and_speed(desired_steering, desired_speed);
//       calc_throttle(desired_speed, true);
//     }

//     if (!is_zero(desired_steering)) {
//       g2.sailboat.clear_tack();
//     }

//     float steering_out = 0.0f;
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

// UPDATE V5 Hệ thống tiết kiệm năng lượng, điều chỉnh theo %throttle và có
// Watchdog
// void ModeAcro::update() {
//   const uint32_t now_ms = AP_HAL::millis();
//   const float dt = rover.G_Dt;

//   if ((dt < 0.0001f) || (dt > 0.1f)) {
//     return;
//   }

//   const int8_t rail_en = static_cast<int8_t>(rover.g.rail_enable.get());
//   const bool currently_armed = rover.arming.is_armed();
//   const bool just_armed = (currently_armed && (rail_state_.last_armed == 0));
//   const bool new_session = ((now_ms - rail_state_.last_update_ms) > 300U);

//   rail_state_.last_update_ms = now_ms;
//   rail_state_.last_armed = currently_armed ? 1 : 0;

//   if (rail_en == 1) {
//     const float target_v_param = rover.g.rail_speed.get();

//     if ((rail_en != rail_state_.last_enable) ||
//         !is_equal(target_v_param, rail_state_.last_speed)) {
//       if (rail_state_.last_enable != 1) {
//         rail_state_.ramped_speed = 0.0f;
//       }
//       rail_state_.target_speed_reached_ms = 0U;
//       rail_state_.rail_phase1_start_ms = now_ms;
//       rail_state_.duration_timeout_triggered = false;
//       rail_state_.safety_timeout_active = false;
//       rail_state_.captured_throttle = 0.0f;

//       gcs().send_text(MAV_SEVERITY_INFO,
//                       "[RAIL] Rail:ACTIVE | Target: %.3f m/s",
//                       static_cast<double>(target_v_param));
//       rail_state_.last_enable = rail_en;
//       rail_state_.last_speed = target_v_param;
//     }

//     float actual_speed = 0.0f;
//     const bool speed_available =
//         attitude_control.get_forward_speed(actual_speed);

//     if (just_armed || new_session) {
//       rail_state_.ramped_speed = speed_available ? actual_speed : 0.0f;
//       rail_state_.pitch_safe_start_ms = 0U;
//       rail_state_.target_speed_reached_ms = 0U;
//       rail_state_.rail_phase1_start_ms = now_ms;
//       rail_state_.duration_timeout_triggered = false;
//       rail_state_.safety_timeout_active = false;
//       rail_state_.captured_throttle = 0.0f;
//     }

//     float target_v = target_v_param;
//     const float allowed_speed_error =
//         constrain_float(rover.g.rail_speed_error.get(), 0.02f, 1.0f);

//     // --- KIỂM TRA HỒI TRẢ TRẠNG THÁI (LATCH RELEASE) KHI MẤT ỔN ĐỊNH ---
//     if (rail_state_.duration_timeout_triggered && speed_available) {
//       const float active_target =
//           rail_state_.safety_timeout_active
//               ? (target_v_param *
//                  (constrain_int16(rover.g.rail_percent.get(), 10, 90) *
//                  0.01f))
//               : target_v_param;

//       if (fabsf(actual_speed - active_target) > (allowed_speed_error * 1.5f))
//       {
//         rail_state_.duration_timeout_triggered = false;
//         rail_state_.target_speed_reached_ms = 0U;
//         rail_state_.safety_timeout_active = false;
//         rail_state_.rail_phase1_start_ms = now_ms;
//         rail_state_.captured_throttle = 0.0f;

//         gcs().send_text(
//             MAV_SEVERITY_WARNING,
//             "[RAIL] UNSTABLE! Reset Watchdog & Re-engaging PID Loop");
//       }
//     }

//     // --- LUỒNG STATE MACHINE QUẢN LÝ ĐẦU RA MOTOR ---
//     if (rail_state_.duration_timeout_triggered) {
//       g2.motors.set_throttle(rail_state_.captured_throttle);
//     } else {
//       const float safety_limit_sec = rover.g.rail_safety_timeout.get();
//       if ((safety_limit_sec > 0.0f) && (!rail_state_.safety_timeout_active))
//       {
//         if ((now_ms - rail_state_.rail_phase1_start_ms) >=
//             static_cast<uint32_t>(safety_limit_sec * 1000.0f)) {
//           rail_state_.safety_timeout_active = true;
//           const int8_t percent_raw =
//               constrain_int16(rover.g.rail_percent.get(), 10, 90);
//           const float rail_scale = static_cast<float>(percent_raw) * 0.01f;

//           gcs().send_text(MAV_SEVERITY_CRITICAL,
//                           "[RAIL] WATCHDOG Reduced: %d%% | New Speed:
//                           %.3fm/s", static_cast<int>(100 - percent_raw),
//                           static_cast<double>(target_v_param * rail_scale));
//         }
//       }

//       if (rail_state_.safety_timeout_active) {
//         const int8_t percent_raw =
//             constrain_int16(rover.g.rail_percent.get(), 10, 90);
//         target_v = target_v_param * (static_cast<float>(percent_raw) *
//         0.01f);
//       }

//       if (rover.g.rail_safe_pitch_en.get() == 1) {
//         const float pitch_deg = degrees(rover.ahrs.get_pitch());
//         const float current_pitch_rate_deg_s =
//         degrees(rover.ahrs.get_gyro().y);

//         const float raw_pitch_accel_deg_s2 =
//             (current_pitch_rate_deg_s - rail_state_.last_pitch_rate_deg_s) /
//             dt;
//         rail_state_.last_pitch_rate_deg_s = current_pitch_rate_deg_s;

//         const float lpf_alpha = constrain_float(dt / (0.04f + dt),
//         0.05f, 1.0f); rail_state_.filtered_pitch_accel_deg_s2 =
//             (lpf_alpha * raw_pitch_accel_deg_s2) +
//             ((1.0f - lpf_alpha) * rail_state_.filtered_pitch_accel_deg_s2);

//         const float safe_pitch_down_limit =
//             -fabsf(rover.g.safe_pitch_down.get());
//         const float safe_pitch_up_limit = fabsf(rover.g.safe_pitch_up.get());
//         const float safe_pitch_accel_limit =
//             fabsf(rover.g.safe_pitch_accel.get());

//         const bool is_pitch_bad =
//             (pitch_deg < safe_pitch_down_limit) ||
//             (pitch_deg > safe_pitch_up_limit) ||
//             (fabsf(rail_state_.filtered_pitch_accel_deg_s2) >
//              safe_pitch_accel_limit);

//         const float pitch_scale =
//             constrain_float(rover.g.rail_pitch_scale.get() * 0.01f,
//             0.0f, 1.0f);

//         if (is_pitch_bad) {
//           const float max_allowed_speed = target_v_param * pitch_scale;
//           if (target_v > max_allowed_speed) {
//             target_v = max_allowed_speed;
//           }
//           rail_state_.pitch_safe_start_ms = 0U;
//           rail_state_.target_speed_reached_ms = 0U;

//           if (!rail_state_.pitch_warning_sent) {
//             gcs().send_text(
//                 MAV_SEVERITY_CRITICAL,
//                 "[RAIL] PITCH DANGER! Ang:%.2fdeg Acc:%.2fdeg/s2",
//                 static_cast<double>(pitch_deg),
//                 static_cast<double>(rail_state_.filtered_pitch_accel_deg_s2));
//             rail_state_.pitch_warning_sent = true;
//           }
//         } else if (rail_state_.pitch_warning_sent) {
//           if (rail_state_.pitch_safe_start_ms == 0U) {
//             rail_state_.pitch_safe_start_ms = now_ms;
//           }
//           if ((now_ms - rail_state_.pitch_safe_start_ms) >=
//               static_cast<uint32_t>(
//                   MAX(rover.g.rail_pitch_delay.get(), 0.0f))) {
//             gcs().send_text(MAV_SEVERITY_WARNING,
//                             "[RAIL] Pitch & Inertia Safe - Resuming");
//             rail_state_.pitch_warning_sent = false;
//             rail_state_.pitch_safe_start_ms = 0U;
//           } else {
//             target_v *= pitch_scale;
//             rail_state_.target_speed_reached_ms = 0U;
//           }
//         }
//       } else {
//         rail_state_.last_pitch_rate_deg_s = 0.0f;
//         rail_state_.filtered_pitch_accel_deg_s2 = 0.0f;
//       }

//       if (speed_available) {
//         const float accel_step = rover.g.rail_ramped_rate.get() * dt;
//         const float spd_lead = MAX(rover.g.rail_speed_lead.get(), 0.1f);

//         rail_state_.ramped_speed =
//             constrain_float(rail_state_.ramped_speed, actual_speed -
//             spd_lead,
//                             actual_speed + spd_lead);

//         if (rail_state_.ramped_speed < target_v) {
//           rail_state_.ramped_speed =
//               MIN(target_v, rail_state_.ramped_speed + accel_step);
//         } else if (rail_state_.ramped_speed > target_v) {
//           rail_state_.ramped_speed =
//               MAX(target_v, rail_state_.ramped_speed - accel_step);
//         }

//         calc_throttle(rail_state_.ramped_speed, false);

//         const float speed_error = fabsf(actual_speed - target_v);
//         const float duration_limit_sec = rover.g.rail_duration.get();

//         if ((duration_limit_sec > 0.0f) &&
//             (speed_error <= allowed_speed_error) &&
//             (!rail_state_.pitch_warning_sent) &&
//             (!rail_state_.safety_timeout_active)) {
//           if (rail_state_.target_speed_reached_ms == 0U) {
//             rail_state_.target_speed_reached_ms = now_ms;
//           }

//           if ((now_ms - rail_state_.target_speed_reached_ms) >=
//               static_cast<uint32_t>(duration_limit_sec * 1000.0f)) {
//             const int8_t reduction_raw =
//                 constrain_int16(rover.g.rail_throttle_reduction.get(), 0,
//                 50);
//             const float calculated_throt =
//                 g2.motors.get_throttle() -
//                 (g2.motors.get_throttle() *
//                  (static_cast<float>(reduction_raw) * 0.01f));

//             rail_state_.captured_throttle =
//                 constrain_float(calculated_throt, 0.0f, 100.0f);
//             rail_state_.duration_timeout_triggered = true;

//             gcs().send_text(MAV_SEVERITY_WARNING,
//                             "[RAIL] STABLE! Locked (Reduced %d%%) at: %d%%",
//                             static_cast<int>(reduction_raw),
//                             static_cast<int>(rail_state_.captured_throttle));
//           }
//         } else {
//           rail_state_.target_speed_reached_ms = 0U;
//         }
//       } else {
//         g2.motors.set_throttle(0.0f);
//         set_steering(0.0f);
//         gcs().send_text(MAV_SEVERITY_EMERGENCY,
//                         "[RAIL] ERROR: EKF Speed Lost!");
//         return;
//       }
//     }

//     float p_steer = 0.0f;
//     float p_throt = 0.0f;
//     get_pilot_desired_steering_and_throttle(p_steer, p_throt);
//     float target_turn_rad = 0.0f;
//     const float steer_dz = static_cast<float>(rover.g.rail_steer_dz.get());

//     if ((rover.g.rail_auto_steer.get() == 1) && (fabsf(p_steer) > steer_dz))
//     {
//       target_turn_rad = (p_steer > 0.0f)
//                             ? radians(rover.g.rail_auto_turn_rate.get())
//                             : -radians(rover.g.rail_auto_turn_rate.get());
//     } else if (fabsf(p_steer) > steer_dz) {
//       target_turn_rad = (p_steer / 4500.0f) * radians(g2.acro_turn_rate);
//     }

//     const float steer_out = attitude_control.get_steering_out_rate(
//         target_turn_rad, g2.motors.limit.steer_left,
//         g2.motors.limit.steer_right, dt);
//     set_steering(constrain_float(steer_out, -1.0f, 1.0f) * 4500.0f);

//     if ((rover.g.rail_log_enable.get() == 1) &&
//         ((now_ms - rail_state_.last_log_ms) > 2000U)) {
//       if (!rail_state_.duration_timeout_triggered) {
//         gcs().send_text(MAV_SEVERITY_INFO,
//                         "[RAIL] TRACK_SPEED | Act:%.3f | Tgt:%.3f |
//                         Thr:%d%%", static_cast<double>(actual_speed),
//                         static_cast<double>(target_v),
//                         static_cast<int>(g2.motors.get_throttle()));
//       } else {
//         gcs().send_text(MAV_SEVERITY_INFO,
//                         "[RAIL] LOCKED_THROT | Act:%.3f | LckThr:%d%%",
//                         static_cast<double>(actual_speed),
//                         static_cast<int>(rail_state_.captured_throttle));
//       }
//       rail_state_.last_log_ms = now_ms;
//     }

//   } else {
//     if (rail_state_.last_enable == 1) {
//       gcs().send_text(MAV_SEVERITY_INFO, "[RAIL] Rail:DEACTIVATE");
//       rail_state_.last_enable = 0;
//       rail_state_.pitch_warning_sent = false;
//       rail_state_.pitch_safe_start_ms = 0U;
//       rail_state_.target_speed_reached_ms = 0U;
//       rail_state_.rail_phase1_start_ms = 0U;
//       rail_state_.duration_timeout_triggered = false;
//       rail_state_.safety_timeout_active = false;
//       rail_state_.captured_throttle = 0.0f;
//     }

//     float speed = 0.0f;
//     float desired_steering = 0.0f;
//     if (!attitude_control.get_forward_speed(speed)) {
//       float desired_throttle = 0.0f;
//       get_pilot_desired_steering_and_throttle(desired_steering,
//                                               desired_throttle);
//       if (rover.is_balancebot()) {
//         rover.balancebot_pitch_control(desired_throttle);
//       }
//       g2.motors.set_throttle(desired_throttle);
//     } else {
//       float desired_speed = 0.0f;
//       get_pilot_desired_steering_and_speed(desired_steering, desired_speed);
//       calc_throttle(desired_speed, true);
//     }

//     if (!is_zero(desired_steering)) {
//       g2.sailboat.clear_tack();
//     }

//     float steering_out = 0.0f;
//     if (g2.sailboat.tacking()) {
//       steering_out = attitude_control.get_steering_out_heading(
//           g2.sailboat.get_tack_heading_rad(), g2.wp_nav.get_pivot_rate(),
//           g2.motors.limit.steer_left, g2.motors.limit.steer_right, dt);
//     } else {
//       steering_out = attitude_control.get_steering_out_rate(
//           (desired_steering / 4500.0f) * radians(g2.acro_turn_rate),
//           g2.motors.limit.steer_left, g2.motors.limit.steer_right, dt);
//     }
//     set_steering(steering_out * 4500.0f);
//   }
// }
// UPDATE V6.0
void ModeAcro::update() {
  const uint32_t now_ms = AP_HAL::millis();
  const float dt = rover.G_Dt;

  // --- INPUT VALIDATION STAGE ---
  if ((dt < 0.0001f) || (dt > 0.1f)) {
    return;
  }

  const int8_t rail_en = static_cast<int8_t>(rover.g.rail_enable.get());
  const bool currently_armed = rover.arming.is_armed();
  const bool just_armed = (currently_armed && (rail_state_.last_armed == 0));
  const bool new_session = ((now_ms - rail_state_.last_update_ms) > 300U);

  rail_state_.last_update_ms = now_ms;
  rail_state_.last_armed = currently_armed ? 1 : 0;

  // --- BRANCH SEPARATION: RAIL MODE ACTIVE vs TRADITIONAL BYPASS ---
  if (rail_en == 1) {
    const float target_v_param = rover.g.rail_speed.get();
    const float duration_limit_config = rover.g.rail_duration.get();
    const float duration_limit_sec =
        (duration_limit_config > 1.0f) ? duration_limit_config : 1.0f;

    // Reset cờ trạng thái khi cấu hình tham số thay đổi từ GCS
    if ((rail_en != rail_state_.last_enable) ||
        (fabsf(target_v_param - rail_state_.last_speed) > 0.0001f)) {
      if (rail_state_.last_enable != 1) {
        rail_state_.ramped_speed = 0.0f;
      }
      rail_state_.rail_phase1_start_ms = now_ms;
      rail_state_.duration_timeout_triggered = false;
      rail_state_.target_speed_reached = false;
      rail_state_.stable_start_ms = 0U;
      rail_state_.captured_throttle = 0.0f;
      rail_state_.filtered_throttle = 0.0f;
      rail_state_.last_ema_throttle = 0.0f;

      gcs().send_text(MAV_SEVERITY_INFO,
                      "[RAIL] System Active | Target: %.2f m/s",
                      static_cast<double>(target_v_param));
      rail_state_.last_enable = rail_en;
      rail_state_.last_speed = target_v_param;
    }

    float actual_speed = 0.0f;
    const bool speed_available =
        rover.g2.attitude_control.get_forward_speed(actual_speed);

    if (just_armed || new_session) {
      rail_state_.ramped_speed = speed_available ? actual_speed : 0.0f;
      rail_state_.pitch_safe_start_ms = 0U;
      rail_state_.rail_phase1_start_ms = now_ms;
      rail_state_.duration_timeout_triggered = false;
      rail_state_.target_speed_reached = false;
      rail_state_.stable_start_ms = 0U;
      rail_state_.captured_throttle = 0.0f;
      rail_state_.filtered_throttle = g2.motors.get_throttle();
      rail_state_.last_ema_throttle = rail_state_.filtered_throttle;
    }

    const float allowed_speed_error =
        constrain_float(rover.g.rail_speed_error.get(), 0.02f, 1.0f);

    // --- MONITOR & LATCH RELEASE BREAKOUT ---
    if (rail_state_.duration_timeout_triggered && speed_available) {
      if (fabsf(actual_speed - target_v_param) > (allowed_speed_error * 1.5f)) {
        rail_state_.duration_timeout_triggered = false;
        rail_state_.target_speed_reached = false; // Bẻ chốt buộc phải đề ba lại
        rail_state_.rail_phase1_start_ms = now_ms;
        rail_state_.stable_start_ms = 0U;
        rail_state_.captured_throttle = 0.0f;
        rail_state_.filtered_throttle = g2.motors.get_throttle();
        rail_state_.last_ema_throttle = rail_state_.filtered_throttle;

        gcs().send_text(
            MAV_SEVERITY_WARNING,
            "[RAIL] SPEED UNSTABLE! Breaking Latch for PID Recovery");
      }
    }

    // --- MAIN CONTROL ENGINE (STATE MACHINE) ---
    if (rail_state_.duration_timeout_triggered) {
      g2.motors.set_throttle(rail_state_.captured_throttle);
    } else {
      float target_v = target_v_param;

      // [Khối giám sát an toàn cơ học Pitch]
      if (rover.g.rail_safe_pitch_en.get() == 1) {
        const float pitch_deg = degrees(rover.ahrs.get_pitch());
        const float current_pitch_rate_deg_s = degrees(rover.ahrs.get_gyro().y);
        const float raw_pitch_accel_deg_s2 =
            (current_pitch_rate_deg_s - rail_state_.last_pitch_rate_deg_s) / dt;
        rail_state_.last_pitch_rate_deg_s = current_pitch_rate_deg_s;

        const float LPF_ALPHA = constrain_float(dt / (0.04f + dt), 0.05f, 1.0f);
        rail_state_.filtered_pitch_accel_deg_s2 =
            (LPF_ALPHA * raw_pitch_accel_deg_s2) +
            ((1.0f - LPF_ALPHA) * rail_state_.filtered_pitch_accel_deg_s2);

        const float safe_pitch_down_limit =
            -fabsf(rover.g.safe_pitch_down.get());
        const float safe_pitch_up_limit = fabsf(rover.g.safe_pitch_up.get());
        const float safe_pitch_accel_limit =
            fabsf(rover.g.safe_pitch_accel.get());

        const bool is_pitch_bad =
            (pitch_deg < safe_pitch_down_limit) ||
            (pitch_deg > safe_pitch_up_limit) ||
            (fabsf(rail_state_.filtered_pitch_accel_deg_s2) >
             safe_pitch_accel_limit);
        const float pitch_scale =
            constrain_float(rover.g.rail_pitch_scale.get() * 0.01f, 0.0f, 1.0f);

        if (is_pitch_bad) {
          const float max_allowed_speed = target_v_param * pitch_scale;
          if (target_v > max_allowed_speed) {
            target_v = max_allowed_speed;
          }
          rail_state_.pitch_safe_start_ms = 0U;
          rail_state_.rail_phase1_start_ms = now_ms;

          if (!rail_state_.pitch_warning_sent) {
            gcs().send_text(MAV_SEVERITY_CRITICAL,
                            "[RAIL] CRITICAL PITCH! Restricting Target Speed");
            rail_state_.pitch_warning_sent = true;
          }
        } else if (rail_state_.pitch_warning_sent) {
          if (rail_state_.pitch_safe_start_ms == 0U) {
            rail_state_.pitch_safe_start_ms = now_ms;
          }

          const float pitch_delay_config = rover.g.rail_pitch_delay.get();
          const float safe_delay_sec =
              (pitch_delay_config > 0.0f) ? pitch_delay_config : 0.0f;

          if ((now_ms - rail_state_.pitch_safe_start_ms) >=
              static_cast<uint32_t>(safe_delay_sec * 1000.0f)) {
            rail_state_.pitch_warning_sent = false;
            rail_state_.pitch_safe_start_ms = 0U;
            rail_state_.rail_phase1_start_ms = now_ms;
          } else {
            target_v *= pitch_scale;
          }
        }
      }

      if (speed_available) {
        const float accel_step = rover.g.rail_ramped_rate.get() * dt;
        const float speed_lead_config = rover.g.rail_speed_lead.get();
        const float spd_lead =
            (speed_lead_config > 0.1f) ? speed_lead_config : 0.1f;

        rail_state_.ramped_speed =
            constrain_float(rail_state_.ramped_speed, actual_speed - spd_lead,
                            actual_speed + spd_lead);

        if (rail_state_.ramped_speed < target_v) {
          const float next_speed = rail_state_.ramped_speed + accel_step;
          rail_state_.ramped_speed =
              (next_speed < target_v) ? next_speed : target_v;
        } else if (rail_state_.ramped_speed > target_v) {
          const float next_speed = rail_state_.ramped_speed - accel_step;
          rail_state_.ramped_speed =
              (next_speed > target_v) ? next_speed : target_v;
        }

        // LUÔN LUÔN CHẠY PID VÒNG KÍN KHI CHƯA CHỐT GA TĨNH
        calc_throttle(rail_state_.ramped_speed, false);

        // --- GIAI ĐOẠN ĐỀ BA (TAKEOFF PHASE):
        if (!rail_state_.target_speed_reached) {
          rail_state_.rail_phase1_start_ms = now_ms;
          rail_state_.stable_start_ms =
              0U; // Khóa cứng bộ đếm ổn định bằng 0 khi đang đề ba

          if (actual_speed >= (target_v_param - allowed_speed_error)) {
            rail_state_.target_speed_reached = true;
            rail_state_.stable_start_ms =
                now_ms; // Khởi tạo mốc thời gian đánh giá hội tụ
          }
        }

        // --- GIAI ĐOẠN ĐÁNH GIÁ ỔN ĐỊNH VÀ TỐI ƯU GA (PROFILING PHASE) ---
        // Khối logic này chỉ được phép xử lý sau khi đã hoàn thành đề ba
        if (rail_state_.target_speed_reached) {
          const float time_elapsed_ms =
              static_cast<float>(now_ms - rail_state_.rail_phase1_start_ms);
          const float total_duration_ms = duration_limit_sec * 1000.0f;

          const float calculated_tau = duration_limit_sec * 0.333f;
          const float tau = (calculated_tau > 0.5f) ? calculated_tau : 0.5f;

          const float alpha = constrain_float(dt / (tau + dt), 0.001f, 1.0f);
          rail_state_.filtered_throttle =
              (alpha * g2.motors.get_throttle()) +
              ((1.0f - alpha) * rail_state_.filtered_throttle);

          const float ema_derivative = fabsf(rail_state_.filtered_throttle -
                                             rail_state_.last_ema_throttle) /
                                       dt;
          rail_state_.last_ema_throttle = rail_state_.filtered_throttle;

          const float speed_error = fabsf(actual_speed - target_v);

          const float progress_ratio =
              constrain_float(time_elapsed_ms / total_duration_ms, 0.0f, 1.0f);
          const float epsilon_base = 0.4f;
          const float epsilon_max = 2.5f;
          const float adaptive_epsilon =
              epsilon_base + (progress_ratio * (epsilon_max - epsilon_base));

          const bool is_converged = (ema_derivative < adaptive_epsilon) &&
                                    (speed_error <= allowed_speed_error);

          if (is_converged && (!rail_state_.pitch_warning_sent)) {
            if (rail_state_.stable_start_ms == 0U) {
              rail_state_.stable_start_ms = now_ms;
            }
            if ((now_ms - rail_state_.stable_start_ms) > 500U) {
              // CHỐT GA TỐI ƯU
              const int8_t reduction_raw =
                  constrain_int16(rover.g.rail_throttle_reduction.get(), 0, 80);
              const float optimal_throt =
                  rail_state_.filtered_throttle *
                  (1.0f - static_cast<float>(reduction_raw) * 0.01f);

              rail_state_.captured_throttle =
                  constrain_float(optimal_throt, 0.0f, 100.0f);
              rail_state_.duration_timeout_triggered = true;

              gcs().send_text(MAV_SEVERITY_WARNING,
                              "[RAIL] Latch Engaged at %d%% | PID Suspended",
                              static_cast<int>(rail_state_.captured_throttle));
            }
          } else {
            rail_state_.stable_start_ms =
                0U; // Reset bộ đếm nếu xuất hiện dao động vượt biên thích nghi
          }
        } else {
          // Bảo bám (Tracking) bộ lọc EMA liên tục theo PID thô khi chưa đề ba
          // xong
          rail_state_.filtered_throttle = g2.motors.get_throttle();
          rail_state_.last_ema_throttle = rail_state_.filtered_throttle;
        }
      } else {
        g2.motors.set_throttle(0.0f);
        set_steering(0.0f);
        gcs().send_text(MAV_SEVERITY_EMERGENCY,
                        "[RAIL] LOSS OF EKF SPEED DATA! Emergency Stop");
        return;
      }
    }

    // --- MIXER XỬ LÝ HƯỚNG LÁI VI SẠI TỰ ĐỘNG KHÔNG ĐỔI ---
    float p_steer = 0.0f;
    float p_throt = 0.0f;
    get_pilot_desired_steering_and_throttle(p_steer, p_throt);
    float target_turn_rad = 0.0f;
    const float steer_dz = static_cast<float>(rover.g.rail_steer_dz.get());

    if ((rover.g.rail_auto_steer.get() == 1) && (fabsf(p_steer) > steer_dz)) {
      target_turn_rad = (p_steer > 0.0f)
                            ? radians(rover.g.rail_auto_turn_rate.get())
                            : -radians(rover.g.rail_auto_turn_rate.get());
    } else if (fabsf(p_steer) > steer_dz) {
      target_turn_rad = (p_steer / 4500.0f) * radians(g2.acro_turn_rate);
    }

    const float steer_out = rover.g2.attitude_control.get_steering_out_rate(
        target_turn_rad, g2.motors.limit.steer_left,
        g2.motors.limit.steer_right, dt);
    set_steering(constrain_float(steer_out, -1.0f, 1.0f) * 4500.0f);

    if ((rover.g.rail_log_enable.get() == 1) &&
        ((now_ms - rail_state_.last_log_ms) > 2000U)) {
      gcs().send_text(MAV_SEVERITY_INFO,
                      "[RAIL] Mon | Vel:%.2f | EmaThr:%d%% | Latch:%d",
                      static_cast<double>(actual_speed),
                      static_cast<int>(rail_state_.filtered_throttle),
                      rail_state_.duration_timeout_triggered ? 1 : 0);
      rail_state_.last_log_ms = now_ms;
    }

  } else {
    // --- FALLBACK TO TRADITIONAL ACRO MODE ---
    if (rail_state_.last_enable == 1) {
      gcs().send_text(MAV_SEVERITY_INFO, "[RAIL] Rail Mode: Deactivated");
      rail_state_.last_enable = 0;
      rail_state_.pitch_warning_sent = false;
      rail_state_.pitch_safe_start_ms = 0U;
      rail_state_.rail_phase1_start_ms = 0U;
      rail_state_.duration_timeout_triggered = false;
      rail_state_.target_speed_reached = false;
      rail_state_.stable_start_ms = 0U;
      rail_state_.captured_throttle = 0.0f;
      rail_state_.filtered_throttle = 0.0f;
      rail_state_.last_ema_throttle = 0.0f;
    }

    float speed = 0.0f;
    float desired_steering = 0.0f;
    if (!rover.g2.attitude_control.get_forward_speed(speed)) {
      float desired_throttle = 0.0f;
      get_pilot_desired_steering_and_throttle(desired_steering,
                                              desired_throttle);
      g2.motors.set_throttle(desired_throttle);
    } else {
      float desired_speed = 0.0f;
      get_pilot_desired_steering_and_speed(desired_steering, desired_speed);
      calc_throttle(desired_speed, true);
    }

    if (!is_zero(desired_steering)) {
      g2.sailboat.clear_tack();
    }

    float steering_out = 0.0f;
    if (g2.sailboat.tacking()) {
      steering_out = rover.g2.attitude_control.get_steering_out_heading(
          g2.sailboat.get_tack_heading_rad(), g2.wp_nav.get_pivot_rate(),
          g2.motors.limit.steer_left, g2.motors.limit.steer_right, dt);
    } else {
      steering_out = rover.g2.attitude_control.get_steering_out_rate(
          (desired_steering / 4500.0f) * radians(g2.acro_turn_rate),
          g2.motors.limit.steer_left, g2.motors.limit.steer_right, dt);
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