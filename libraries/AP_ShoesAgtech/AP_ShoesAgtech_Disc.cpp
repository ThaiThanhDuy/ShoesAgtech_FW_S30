// =============================================================
// MODULE 3 (rut gon) — DIA RAI LY TAM
// Clone tu Module 3 (Dosing) ban S16, CHI giu phan dia rai: KHONG co
// truc vit (SA_DOS_CHAN/MODE/SP/FOOD/Ax/Bx), KHONG co SA_DISC_DLY (chi
// dung de canh thoi gian dia <-> truc vit), KHONG phu thuoc ao.
// Chi so tham so giu NGUYEN nhu S16 (subgroup 4) de file .param tuong
// thich.
// =============================================================
#include "AP_ShoesAgtech.h"
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>

const AP_Param::GroupInfo AP_ShoesAgtech_DiscParams::var_info[] = {
    // @Param: DOS_RC
    // @DisplayName: RC channel to toggle the spreader disc on/off (1-indexed)
    // @Description: PWM>1500 runs the disc; PWM<=1500 stops it.
    // @Range: 1 16
    // @User: Standard
    AP_GROUPINFO("DOS_RC", 2, AP_ShoesAgtech_DiscParams, dos_rc, 8),

    // @Param: DOS_LOG
    // @DisplayName: Spreader disc console log enable
    // @Description: Prints disc state and PWM at SA_DOS_LOG_MS interval.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("DOS_LOG", 5, AP_ShoesAgtech_DiscParams, log_enable, 0),

    // @Param: DOS_LOG_MS
    // @DisplayName: Spreader disc console log interval (ms)
    // @Range: 100 60000
    // @Units: ms
    // @User: Advanced
    AP_GROUPINFO("DOS_LOG_MS", 6, AP_ShoesAgtech_DiscParams, log_ms, 2000),

    // @Param: DISC_CHAN
    // @DisplayName: Centrifugal spreader disc ESC output channel (1-indexed)
    // @Description: Servo/ESC channel driving the spreader disc motor. 0
    //   disables this feature entirely (no PWM ever written for the disc).
    //   Requires SERVOx_FUNCTION=0(None) and SERVOx_MAX=2200.
    // @Range: 0 16
    // @User: Standard
    AP_GROUPINFO("DISC_CHAN", 24, AP_ShoesAgtech_DiscParams, disc_chan, 0),

    // @Param: DISC_PCT
    // @DisplayName: Spreader disc speed when running (% of PWM range)
    // @Description: 100 = full speed, 0 = stop point. See SA_DISC_REV for
    //   which PWM range the percentage is applied to.
    // @Range: 0 100
    // @Units: %
    // @User: Standard
    AP_GROUPINFO("DISC_PCT", 25, AP_ShoesAgtech_DiscParams, disc_pct, 100),

    // @Param: DISC_REV
    // @DisplayName: Spreader disc ESC type / direction
    // @Description: 0=StandardESC: one-way ESC, 0%=SA_DISC_MIN,
    //   100%=SERVOx_MAX, no TRIM check. 1=BidirForward: two-way ESC stopped
    //   at TRIM(1500), 100%=MAX(2200); requires MIN=800/TRIM=1500/MAX=2200.
    //   2=BidirReverse: like BidirForward but 100%=MIN(800).
    // @Values: 0:StandardESC,1:BidirForward,2:BidirReverse
    // @Range: 0 2
    // @User: Standard
    AP_GROUPINFO("DISC_REV", 41, AP_ShoesAgtech_DiscParams, disc_rev, 0),

    // @Param: DISC_MIN
    // @DisplayName: Spreader disc StandardESC min PWM
    // @Description: PWM for 0% disc speed (and stop) when SA_DISC_REV=0,
    //   used instead of the channel's real SERVOx_MIN. No effect when
    //   SA_DISC_REV=1/2.
    // @Range: 800 2200
    // @Units: PWM
    // @User: Standard
    AP_GROUPINFO("DISC_MIN", 42, AP_ShoesAgtech_DiscParams, disc_min, 1100),

    AP_GROUPEND};

// =============================================================
// KIEM TRA CAU HINH KENH DIA RAI (giu nguyen logic S16)
// Luon bat buoc FUNCTION=0(None) va MAX=2200. MIN=800/TRIM=1500 chi bat
// buoc khi SA_DISC_REV=1/2 (ESC 2 chieu). Sai -> canh bao moi 5s.
// =============================================================
void AP_ShoesAgtech::_check_disc_config(void) {
  if (_disc_params.disc_chan.get() <= 0) {
    _disc_config_ok = false;
    _disc_was_ok = false;
    return;
  }

  const uint8_t chan_idx =
      (uint8_t)constrain_int16(_disc_params.disc_chan.get() - 1, 0, 15);
  SRV_Channel *ch = SRV_Channels::srv_channel(chan_idx);
  const int32_t func_val = (int32_t)SRV_Channels::channel_function(chan_idx);
  const int32_t chan = (int32_t)_disc_params.disc_chan.get();
  const bool bidir = (_disc_params.disc_rev.get() != 0);

  const bool have_chan = (ch != nullptr);
  const bool func_ok = have_chan && (func_val == (int32_t)SRV_Channel::k_none);
  const bool max_ok = have_chan && (ch->get_output_max() == 2200);
  const bool min_ok = !bidir || (have_chan && (ch->get_output_min() == 800));
  const bool trim_ok = !bidir || (have_chan && (ch->get_trim() == 1500));

  _disc_config_ok = func_ok && min_ok && trim_ok && max_ok;

  if (_disc_config_ok) {
    if (!_disc_was_ok) {
      gcs().send_text(MAV_SEVERITY_INFO,
                      "SA: SERVO%d setup OK - spreader disc ready", (int)chan);
    }
    _disc_was_ok = true;
    return;
  }
  _disc_was_ok = false;

  const uint32_t now = AP_HAL::millis();
  if (now - _disc_warn_ms < 5000) {
    return;
  }
  _disc_warn_ms = now;

  if (!have_chan) {
    gcs().send_text(MAV_SEVERITY_WARNING, "SA: SERVO%d does not exist",
                    (int)chan);
    return;
  }
  if (!func_ok) {
    gcs().send_text(MAV_SEVERITY_WARNING,
                    "SA: SERVO%d FUNCTION=%d, must set =0 (None)", (int)chan,
                    (int)func_val);
  }
  if (!min_ok) {
    gcs().send_text(MAV_SEVERITY_WARNING, "SA: SERVO%d MIN=%u, must set =800",
                    (int)chan, (unsigned)ch->get_output_min());
  }
  if (!trim_ok) {
    gcs().send_text(MAV_SEVERITY_WARNING, "SA: SERVO%d TRIM=%u, must set =1500",
                    (int)chan, (unsigned)ch->get_trim());
  }
  if (!max_ok) {
    gcs().send_text(MAV_SEVERITY_WARNING, "SA: SERVO%d MAX=%u, must set =2200",
                    (int)chan, (unsigned)ch->get_output_max());
  }
}

// =============================================================
// DIEU KHIEN DIA RAI — goi tu update() moi chu ky 10Hz
// SA_DOS_RC > 1500 -> dia quay voi SA_DISC_PCT, <= 1500 -> dung.
// SA_DISC_CHAN = 0 -> tat han tinh nang (khong ghi PWM ra kenh nao).
// =============================================================
void AP_ShoesAgtech::_update_disc(void) {
  _check_disc_config();
  if (_disc_params.disc_chan.get() <= 0) {
    _disc_running = false;
    return;
  }

  const uint32_t now = AP_HAL::millis();
  const uint8_t rc_idx =
      (uint8_t)constrain_int16(_disc_params.dos_rc.get() - 1, 0, 15);
  const uint16_t rc_pwm = RC_Channels::get_radio_in(rc_idx);
  bool disc_on = (rc_pwm > 1500);

  // An toan khoi dong (giu nhu S16): dia ly tam quay nhanh -> sau khi bat
  // nguon, cong tac SA_DOS_RC phai ve OFF (<=1500) it nhat 1 lan thi dia
  // moi duoc phep quay. Kiem tra moi 3s, chi can dat 1 lan moi phien nguon.
  if (!_disc_rc_seen_off) {
    if (now - _disc_rc_check_ms >= 3000) {
      _disc_rc_check_ms = now;
      if (rc_pwm > 0 && rc_pwm <= 1500) {
        _disc_rc_seen_off = true;
        gcs().send_text(MAV_SEVERITY_INFO, "SA: Disc RC ready - disc can start");
      } else {
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "SA: Move SA_DOS_RC to OFF to start disc");
      }
    }
    disc_on = false;
  }

  if (disc_on != _disc_was_on) {
    _disc_was_on = disc_on;
    gcs().send_text(MAV_SEVERITY_INFO, "SA: Spreader disc %s",
                    disc_on ? "ON" : "OFF");
  }

  _disc_running = disc_on && _disc_config_ok;

  // Ghi PWM theo SA_DISC_REV:
  //   0 (StandardESC): 0%=SA_DISC_MIN, 100%=SERVOx_MAX. Dung = SA_DISC_MIN.
  //   1 (BidirForward): 0%=TRIM, 100%=MAX. Dung = TRIM.
  //   2 (BidirReverse): 0%=TRIM, 100%=MIN (SERVOx_MIN that). Dung = TRIM.
  const uint8_t disc_idx =
      (uint8_t)constrain_int16(_disc_params.disc_chan.get() - 1, 0, 15);
  SRV_Channel *ch_disc = SRV_Channels::srv_channel(disc_idx);
  if (ch_disc != nullptr) {
    const uint16_t disc_trim = ch_disc->get_trim();
    const uint16_t disc_max = ch_disc->get_output_max();
    const int8_t rev_mode = _disc_params.disc_rev.get();
    const uint16_t disc_min = (rev_mode == 0)
                                  ? (uint16_t)_disc_params.disc_min.get()
                                  : ch_disc->get_output_min();
    if (_disc_running) {
      const float pct =
          constrain_float(_disc_params.disc_pct.get(), 0.0f, 100.0f) * 0.01f;
      if (rev_mode == 0) {
        _disc_pwm =
            (uint16_t)((float)disc_min + pct * (float)(disc_max - disc_min));
      } else if (rev_mode == 1) {
        _disc_pwm =
            (uint16_t)((float)disc_trim + pct * (float)(disc_max - disc_trim));
      } else {
        _disc_pwm =
            (uint16_t)((float)disc_trim - pct * (float)(disc_trim - disc_min));
      }
    } else {
      _disc_pwm = (rev_mode == 0) ? disc_min : disc_trim;
    }
    SRV_Channels::set_output_pwm_chan(disc_idx, _disc_pwm);
  }

  // ---- LOG CONSOLE (SA_DOS_LOG) ----
  if (_disc_params.log_enable.get() > 0 &&
      now - _disc_last_log_ms >= (uint32_t)_disc_params.log_ms.get()) {
    _disc_last_log_ms = now;
    gcs().send_text(MAV_SEVERITY_INFO, "[DISC] %s PWM:%u",
                    _disc_running ? "running" : "not running",
                    (unsigned)_disc_pwm);
  }
}
