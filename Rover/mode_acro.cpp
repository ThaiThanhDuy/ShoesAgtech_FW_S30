#include "Rover.h"

// Update V4 - tối ưu hóa code và clean code
void ModeAcro::update() {
  // === 1. INITIALIZATION ===
  const uint32_t now_ms = AP_HAL::millis();
  const float dt = rover.G_Dt;
  if (dt <= 0.0001f || dt > 0.1f) {
    return;
  }
  const int8_t rail_en = static_cast<int8_t>(rover.g.rail_enable.get());

  const bool currently_armed = rover.arming.is_armed();
  const bool just_armed = (currently_armed && !_rail_last_armed);
  const bool new_session = (now_ms - _rail_last_update_ms > 300U);

  _rail_last_update_ms = now_ms;
  _rail_last_armed = currently_armed;

  // === 2. RAIL MODE LOGIC ===
  if (rail_en == 1) {
    // --- 2.1: State Change Handling ---
    const float target_v_param = rover.g.rail_speed.get();
    if (rail_en != _rail_last_enable ||
        !is_equal(target_v_param, _rail_last_speed)) {
      if (_rail_last_enable != 1) {
        _rail_ramped_speed = 0.0f;
      }
      gcs().send_text(MAV_SEVERITY_INFO, "[RAIL] Rail:ACTIVE | Target: %.2f",
                      static_cast<double>(target_v_param));
      _rail_last_enable = rail_en;
      _rail_last_speed = target_v_param;
    }

    float actual_speed = 0.0f;
    const bool speed_available =
        attitude_control.get_forward_speed(actual_speed);

    // Reset Setpoint to actual speed when starting a new session or Arming
    if (just_armed || new_session) {
      _rail_ramped_speed = speed_available ? actual_speed : 0.0f;
      _rail_pitch_safe_start_ms = 0U; // Reset recovery timer
    }

    // --- 2.2: Pitch Safety Logic (Speed Saturation Limit) ---
    float target_v = target_v_param;
    if (rover.g.rail_safe_pitch_en.get() == 1) {
      // 2.2.1. Trích xuất dữ liệu độ nghiêng tĩnh và vận tốc góc trục Y
      const float pitch_deg = degrees(rover.ahrs.get_pitch());
      const Vector3f &gyro = rover.ahrs.get_gyro();
      const float current_pitch_rate_rads = gyro.y;

      // 2.2.2. Tính toán gia tốc góc Pitch thô (Raw Angular Acceleration)

      float raw_pitch_accel_degs2 = 0.0f;
      if (rover.G_Dt > 0.0001f) {
        raw_pitch_accel_degs2 =
            degrees(current_pitch_rate_rads - _rail_last_pitch_rate_rads) /
            rover.G_Dt;
      }
      _rail_last_pitch_rate_rads =
          current_pitch_rate_rads; // Lưu cấu trúc cho chu kỳ kế tiếp

      // 2.2.3. Áp dụng bộ lọc thông thấp (Low-Pass Filter) tần số cắt ~4Hz

      const float lpf_alpha =
          constrain_float(rover.G_Dt / (0.04f + rover.G_Dt), 0.05f, 1.0f);
      _rail_filtered_pitch_accel_degs2 =
          (lpf_alpha * raw_pitch_accel_degs2) +
          ((1.0f - lpf_alpha) * _rail_filtered_pitch_accel_degs2);

      // 2.2.4. Khởi tạo ngưỡng động học từ hệ thống tham số toàn cục
      const float safe_pitch_down_limit = -fabsf(rover.g.safe_pitch_down.get());
      const float safe_pitch_up_limit = fabsf(rover.g.safe_pitch_up.get());
      const float safe_pitch_accel_limit =
          fabsf(rover.g.safe_pitch_accel.get());

      // 2.2.5. Kiểm tra trạng thái: Vi phạm góc tĩnh HOẶC Vi phạm xung quán
      // tính động lực học
      const bool is_angle_bad = (pitch_deg < safe_pitch_down_limit) ||
                                (pitch_deg > safe_pitch_up_limit);
      const bool is_inertia_bad =
          (fabsf(_rail_filtered_pitch_accel_degs2) > safe_pitch_accel_limit);
      const bool is_pitch_bad = is_angle_bad || is_inertia_bad;

      // Khai báo tỷ lệ scale hỗ trợ dải rộng từ 0% đến 100%
      const float pitch_scale_param = rover.g.rail_pitch_scale.get() * 0.01f;
      const float pitch_scale = constrain_float(pitch_scale_param, 0.0f, 1.0f);

      if (is_pitch_bad) {
        const float max_allowed_speed = target_v_param * pitch_scale;
        if (target_v > max_allowed_speed) {
          target_v = max_allowed_speed;
        }
        _rail_pitch_safe_start_ms =
            0U; // Reset timer phục hồi khi hệ thống vẫn đang vi phạm

        if (!_rail_pitch_warning_sent) {
          gcs().send_text(MAV_SEVERITY_CRITICAL,
                          "[RAIL] PITCH DANGER! Ang:%.1fdeg Acc:%.1fdeg/s2 -> "
                          " LIMIT TO %.2f m/s ",
                          static_cast<double>(pitch_deg),
                          static_cast<double>(_rail_filtered_pitch_accel_degs2),
                          static_cast<double>(max_allowed_speed));
          _rail_pitch_warning_sent = true;
        }
      } else if (_rail_pitch_warning_sent) {
        if (_rail_pitch_safe_start_ms == 0U) {
          _rail_pitch_safe_start_ms = now_ms;
        }

        const uint32_t recovery_delay_ms =
            static_cast<uint32_t>(MAX(rover.g.rail_pitch_delay.get(), 0.0f));

        if (now_ms - _rail_pitch_safe_start_ms >= recovery_delay_ms) {
          gcs().send_text(
              MAV_SEVERITY_WARNING,
              "[RAIL] Pitch & Inertia Safe - Resuming normal speed");
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
    }
    // --- 2.3: Adaptive Ramp Logic ---
    if (speed_available) {
      const float accel_step = rover.g.rail_ramped_rate.get() * dt;
      const float spd_lead = MAX(rover.g.rail_speed_lead.get(), 0.1f);

      // Prevent I-Windup by constraining ramped speed around actual speed
      _rail_ramped_speed = constrain_float(
          _rail_ramped_speed, actual_speed - spd_lead, actual_speed + spd_lead);

      // Smooth profile generator to target_v
      if (_rail_ramped_speed < target_v) {
        _rail_ramped_speed = MIN(target_v, _rail_ramped_speed + accel_step);
      } else if (_rail_ramped_speed > target_v) {
        _rail_ramped_speed = MAX(target_v, _rail_ramped_speed - accel_step);
      }
      calc_throttle(_rail_ramped_speed, false);
    } else {
      // Failsafe: Standard raw duty cycle when EKF feedback is lost
      g2.motors.set_throttle(rover.g.rail_percent * 0.01f);
    }

    // --- 2.4: Steering Process ---
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

    float steer_out = attitude_control.get_steering_out_rate(
        target_turn_rad, g2.motors.limit.steer_left,
        g2.motors.limit.steer_right, dt);
    set_steering(constrain_float(steer_out, -1.0f, 1.0f) * 4500.0f);

    // --- 2.5: Clean Telemetry Logging (2Hz) ---
    if (rover.g.rail_log_enable.get() == 1 &&
        (now_ms - _rail_last_log_ms > 2000U)) {
      gcs().send_text(MAV_SEVERITY_INFO,
                      "[RAIL] T:%.1f R:%.2f A:%.2f STR: %.3f ",
                      static_cast<double>(target_v),
                      static_cast<double>(_rail_ramped_speed),
                      static_cast<double>(actual_speed),
                      static_cast<double>(target_turn_rad));
      _rail_last_log_ms = now_ms;
    }
  } else {
    // === 3. DEFAULT ARDUPILOT ACRO MODE (EXIT RAIL) ===
    if (_rail_last_enable == 1) {
      gcs().send_text(MAV_SEVERITY_INFO, "[RAIL] Rail:DEACTIVATE");
      _rail_last_enable = 0;
      _rail_pitch_warning_sent = false;
      _rail_pitch_safe_start_ms = 0U;
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

// UPDATE V5 Nâng cấp hệ thống tiết kiệm năng lượng và điều chỉnh theo %
// throttle

bool ModeAcro::requires_velocity() const {
  return !g2.motors.have_skid_steering();
}

// sailboats in acro mode support user manually initiating tacking from
// transmitter
void ModeAcro::handle_tack_request() { g2.sailboat.handle_tack_request_acro(); }