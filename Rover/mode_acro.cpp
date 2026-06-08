#include "Rover.h"

// Shoes_Agtech: Rail Mode V6 - 3 pha Takeoff -> Profiling -> Latch (khoa ga on
// dinh) kem Pitch Safety rieng cho Acro/Rail (S30)
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
      rail_state_.last_breakout_ms = 0U;
      rail_state_.throt_window_min = g2.motors.get_throttle();
      rail_state_.throt_window_max = rail_state_.throt_window_min;

      // Canh bao mot lan neu RAIL_DURATION dang bi kep ngam ve toi thieu 1.0s
      if (duration_limit_config <= 1.0f) {
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "[RAIL] RAIL_DURATION %.2fs too low - clamped to 1.0s",
                        static_cast<double>(duration_limit_config));
      }

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
      rail_state_.last_breakout_ms = 0U;
      rail_state_.throt_window_min = rail_state_.filtered_throttle;
      rail_state_.throt_window_max = rail_state_.filtered_throttle;
    }

    const float allowed_speed_error =
        constrain_float(rover.g.rail_speed_error.get(), 0.02f, 1.0f);

    // --- MONITOR & LATCH RELEASE BREAKOUT ---
    // Bo qua breakout khi Pitch Safety dang chu dong giam toc (lech toc do luc
    // nay la CHU Y, khong phai bat on - tranh vong lap Chot <-> Pha chot)
    if (rail_state_.duration_timeout_triggered && speed_available &&
        !rail_state_.pitch_warning_sent) {
      if (fabsf(actual_speed - target_v_param) > (allowed_speed_error * 1.5f)) {
        rail_state_.duration_timeout_triggered = false;
        rail_state_.target_speed_reached = false; // Bẻ chốt buộc phải đề ba lại
        rail_state_.rail_phase1_start_ms = now_ms;
        rail_state_.stable_start_ms = 0U;
        rail_state_.captured_throttle = 0.0f;
        rail_state_.filtered_throttle = g2.motors.get_throttle();
        rail_state_.last_ema_throttle = rail_state_.filtered_throttle;
        rail_state_.last_breakout_ms = now_ms;

        gcs().send_text(
            MAV_SEVERITY_WARNING,
            "[RAIL] SPEED UNSTABLE! Breaking Latch for PID Recovery");
      }
    }

    // --- PITCH SAFETY (chạy mọi pha kể cả latch — Lỗ hổng 1 đã vá) ---
    bool is_pitch_bad = false;
    float pitch_scale_rail = 1.0f;
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

      const float safe_pitch_down_limit = -fabsf(rover.g.safe_pitch_down.get());
      const float safe_pitch_up_limit = fabsf(rover.g.safe_pitch_up.get());
      const float safe_pitch_accel_limit =
          fabsf(rover.g.safe_pitch_accel.get());

      is_pitch_bad = (pitch_deg < safe_pitch_down_limit) ||
                     (pitch_deg > safe_pitch_up_limit) ||
                     (fabsf(rail_state_.filtered_pitch_accel_deg_s2) >
                      safe_pitch_accel_limit);
      pitch_scale_rail =
          constrain_float(rover.g.rail_pitch_scale.get() * 0.01f, 0.0f, 1.0f);

      if (is_pitch_bad) {
        rail_state_.pitch_safe_start_ms = 0U;
        rail_state_.rail_phase1_start_ms = now_ms;
        if (!rail_state_.pitch_warning_sent) {
          gcs().send_text(MAV_SEVERITY_CRITICAL,
                          "[RAIL] CRITICAL PITCH! Restricting Speed/Throttle");
          rail_state_.pitch_warning_sent = true;
        }
      } else if (rail_state_.pitch_warning_sent) {
        if (rail_state_.pitch_safe_start_ms == 0U) {
          rail_state_.pitch_safe_start_ms = now_ms;
        }
        const uint32_t safe_delay_ms =
            static_cast<uint32_t>(MAX(rover.g.rail_pitch_delay.get(), 0));
        if ((now_ms - rail_state_.pitch_safe_start_ms) >= safe_delay_ms) {
          rail_state_.pitch_warning_sent = false;
          rail_state_.pitch_safe_start_ms = 0U;
          rail_state_.rail_phase1_start_ms = now_ms;
        }
      }
    }

    // --- MAIN CONTROL ENGINE (STATE MACHINE) ---
    if (rail_state_.duration_timeout_triggered) {
      // Lỗ hổng 1: pitch safety áp dụng kể cả khi đang chốt ga tĩnh
      float throttle_out = rail_state_.captured_throttle;
      if (rail_state_.pitch_warning_sent) {
        throttle_out *= pitch_scale_rail;
      }
      g2.motors.set_throttle(throttle_out);
    } else {
      float target_v = target_v_param;

      // Áp dụng giới hạn tốc độ từ kết quả pitch safety đã tính ở trên
      if (rover.g.rail_safe_pitch_en.get() == 1) {
        if (is_pitch_bad) {
          const float max_allowed_speed = target_v_param * pitch_scale_rail;
          if (target_v > max_allowed_speed) {
            target_v = max_allowed_speed;
          }
        } else if (rail_state_.pitch_warning_sent) {
          target_v *= pitch_scale_rail;
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
          const float epsilon_base =
              MAX(0.01f, rover.g.rail_epsilon_base.get());
          const float epsilon_max =
              MAX(epsilon_base, rover.g.rail_epsilon_max.get());
          const float adaptive_epsilon =
              epsilon_base + (progress_ratio * (epsilon_max - epsilon_base));

          const bool is_converged = (ema_derivative < adaptive_epsilon) &&
                                    (speed_error <= allowed_speed_error);

          // Cooldown sau breakout - cho dia hinh "lang" lai truoc khi chot lai,
          // tranh vong lap Chot <-> Pha chot lien tuc
          const uint32_t breakout_cooldown_ms = 2000U;
          const bool cooldown_elapsed =
              (rail_state_.last_breakout_ms == 0U) ||
              ((now_ms - rail_state_.last_breakout_ms) >= breakout_cooldown_ms);

          if (is_converged && (!rail_state_.pitch_warning_sent)) {
            if (rail_state_.stable_start_ms == 0U) {
              rail_state_.stable_start_ms = now_ms;
            }
            if (cooldown_elapsed &&
                ((now_ms - rail_state_.stable_start_ms) > 500U)) {
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

    // Theo doi bien dong ga (throttle spread) giua 2 lan log - do luong truc
    // tiep hieu qua "giam dao dong, tiet kiem nang luong" cua Latch
    const float current_throt_for_spread = g2.motors.get_throttle();
    rail_state_.throt_window_min =
        MIN(rail_state_.throt_window_min, current_throt_for_spread);
    rail_state_.throt_window_max =
        MAX(rail_state_.throt_window_max, current_throt_for_spread);

    if ((rover.g.rail_log_enable.get() == 1) &&
        ((now_ms - rail_state_.last_log_ms) > 2000U)) {
      const float throt_spread =
          rail_state_.throt_window_max - rail_state_.throt_window_min;
      gcs().send_text(
          MAV_SEVERITY_INFO,
          "[RAIL] Mon | Vel:%.2f | EmaThr:%d%% | Spread:%d%% | Latch:%d",
          static_cast<double>(actual_speed),
          static_cast<int>(rail_state_.filtered_throttle),
          static_cast<int>(throt_spread),
          rail_state_.duration_timeout_triggered ? 1 : 0);
      rail_state_.last_log_ms = now_ms;

      // Khoi dong lai cua so theo doi cho chu ky ke tiep
      rail_state_.throt_window_min = current_throt_for_spread;
      rail_state_.throt_window_max = current_throt_for_spread;
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
bool ModeAcro::_enter() {
  rail_state_.last_pitch_rate_deg_s = degrees(rover.ahrs.get_gyro().y);
  rail_state_.filtered_pitch_accel_deg_s2 = 0.0f;
  rail_state_.pitch_warning_sent = false;
  rail_state_.pitch_safe_start_ms = 0U;
  return true;
}

bool ModeAcro::requires_velocity() const {
  return !g2.motors.have_skid_steering();
}

// sailboats in acro mode support user manually initiating tacking from
// transmitter
void ModeAcro::handle_tack_request() { g2.sailboat.handle_tack_request_acro(); }