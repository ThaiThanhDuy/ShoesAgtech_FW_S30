#include "AP_ShoesAgtech.h"
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

volatile uint32_t AP_ShoesAgtech::_pulse_count = 0;

// =============================================================
// Chi so (index) giu nguyen nhu S16: ENABLE=1, FLOW subgroup=2,
// DISC subgroup=4 (cung index bang Dosing S16, chi giu cac tham so dia
// rai + SA_DOS_RC/LOG voi dung index & kieu cu), SPD_START=19, SIM=32,
// ROBOT_ID=63. Slot 3 (pH), 59 (POND_IDX) bo trong — KHONG dung lai.
// =============================================================
const AP_Param::GroupInfo AP_ShoesAgtech::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Enable ShoesAgtech library
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("ENABLE", 1, AP_ShoesAgtech, _enable_flag, 1),

    // @Param: ROBOT_ID
    // @DisplayName: Robot identification number
    // @Description: Free-form identifier for this robot/unit. Not used by
    //   the library itself; for external tracking/logging only.
    // @Range: 0 999
    // @User: Standard
    AP_GROUPINFO("ROBOT_ID", 63, AP_ShoesAgtech, _robot_id, 0),

    // @Param: SIM
    // @DisplayName: Simulation mode
    // @Description: When 1, replaces real flow sensor data and ground speed
    //   with sinusoidal test values.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("SIM", 32, AP_ShoesAgtech, _simulation, 0),

    // @Param: SPD_START
    // @DisplayName: Minimum speed to start pumping (% of target mission speed)
    // @Description: FLOW_MODE=1 only starts pumping once ground speed
    //   reaches this percentage of the target mission speed (WP_SPEED,
    //   updated by DO_CHANGE_SPEED/GCS SET_SPEED). 0 disables this check
    //   (any speed above the absolute 0.05 m/s floor starts pumping).
    // @Range: 0 100
    // @Units: %
    // @User: Standard
    AP_GROUPINFO("SPD_START", 19, AP_ShoesAgtech, _spd_start_pct, 80),

    // Module 1 (Bom) — var_info rieng trong AP_ShoesAgtech_Flow.cpp, tien
    // to rong "" nen ten tham so giu nguyen (vd SA_FLOW_MODE).
    AP_SUBGROUPINFO(_flow_params, "", 2, AP_ShoesAgtech, AP_ShoesAgtech_FlowParams),

    // Module 3 rut gon (chi dia rai) — giu index 4 nhu bang Dosing cua S16
    // de SA_DISC_* / SA_DOS_RC tuong thich file .param cu.
    AP_SUBGROUPINFO(_disc_params, "", 4, AP_ShoesAgtech, AP_ShoesAgtech_DiscParams),

    AP_GROUPEND};

// =============================================================
// HAM KHOI TAO (CONSTRUCTOR)
// =============================================================
AP_ShoesAgtech::AP_ShoesAgtech()
    : _last_timestamp_ms(0), _last_pulse_snapshot(0), _last_log_ms(0),
      _flow_rate_filtered(0.0f), _flow_rate_avg(0.0f), _is_initialized(false),
      _buffer_index(0), _buffer_sum(0.0f), _samples_count(0), _spray_mode(0),
      _pump_pwm(0), _flow_target(0.0f), _flow_ramp_val(0.0f),
      _pid_integral(0.0f), _pid_output_lpf(0.0f), _pid_last_ms(0),
      _last_pump_chan(-1), _last_pump_func_val(-1), _pump_config_ok(false),
      _mission_dist_m(0.0f), _mission_ncmds(0),
      _tank_empty_detected(false), _tank_empty_ms(0),
      _no_mission_warned(false), _flow_out_of_range(false),
      _target_speed(0.0f),
      _sim_speed(0.0f), _disc_pwm(0), _disc_running(false),
      _disc_was_on(false), _disc_config_ok(false), _disc_was_ok(false),
      _disc_warn_ms(0), _disc_rc_seen_off(false), _disc_rc_check_ms(0),
      _disc_last_log_ms(0), _init_done(false), _pin_gpio_ok(false),
      _irq_attached(false), _flow_pin_used(0), _diag_warn_ms(0) {
  memset(_sample_buffer, 0, sizeof(_sample_buffer));
  // setup_object_defaults() KHONG tu de quy vao subgroup -> goi rieng cho
  // _flow_params, neu khong cac tham so ben trong se = 0 thay vi default.
  AP_Param::setup_object_defaults(this, var_info);
  AP_Param::setup_object_defaults(&_flow_params, AP_ShoesAgtech_FlowParams::var_info);
  AP_Param::setup_object_defaults(&_disc_params, AP_ShoesAgtech_DiscParams::var_info);
}

// =============================================================
// KHOI DONG (INIT)
// =============================================================
void AP_ShoesAgtech::init(void) {
  if (!is_enabled()) {
    return;
  }

  uint8_t flow_pin = (uint8_t)constrain_int16(_flow_params.flow_pin.get(), 1, 200);
  _flow_pin_used = flow_pin;
  _init_done = true;

  // valid_pin() chi true khi chan AUX da duoc bat lam GPIO
  // (SERVOx_FUNCTION=-1). Neu khong, pinMode() bi bo qua va chan van la
  // ngo ra PWM -> khong dem duoc xung du attach_interrupt() van "thanh cong".
  _pin_gpio_ok = hal.gpio->valid_pin(flow_pin);
  hal.gpio->pinMode(flow_pin, HAL_GPIO_INPUT);

  _irq_attached = hal.gpio->attach_interrupt(flow_pin, irq_handler,
                                             AP_HAL::GPIO::INTERRUPT_RISING);
  if (_irq_attached && _pin_gpio_ok) {
    gcs().send_text(MAV_SEVERITY_INFO, "ShoesAgtech: Flow sensor ready (pin %u)",
                    (unsigned)flow_pin);
  }
  // cac truong hop loi duoc canh bao lap lai trong _flow_diag_check()

  _flow_rate_avg = 0.0f;
  _buffer_index = 0;
  _buffer_sum = 0.0f;
  _samples_count = 0;
  _pid_integral = 0.0f;
  _pid_output_lpf = 0.0f;
  _pid_last_ms = 0;
  memset(_sample_buffer, 0, sizeof(_sample_buffer));
}

void AP_ShoesAgtech::irq_handler(void) { _pulse_count++; }

// =============================================================
// VONG LAP CHINH — goi o tan so 10Hz tu scheduler
// =============================================================
void AP_ShoesAgtech::update(void) {
  if (!is_enabled()) {
    _flow_rate_filtered = 0.0f;
    _flow_rate_avg = 0.0f;
    return;
  }

  _check_pump_config();
  _flow_diag_check();

  if (_simulation.get() > 0) {
    _run_simulation();
  }

  _update_flow();

  _update_disc();
}

// =============================================================
// MO PHONG — SA_SIM = 1
// Sinh luu luong va toc do gia dang song sin de kiem tra mode 1/2 va
// hien thi GCS khi khong gan phan cung that. Buffer trung binh truot van
// do _update_flow() cap nhat nhu binh thuong.
// =============================================================
void AP_ShoesAgtech::_run_simulation(void) {
  const float t = AP_HAL::millis() * 0.001f;

  // luu luong (2.5 ± 1.5 L/min, chu ky 20s)
  _flow_rate_filtered = 2.5f + 1.5f * sinf(2.0f * M_PI * t / 20.0f);

  // toc do mat dat gia lap (1.0 ± 0.8 m/s, chu ky 30s)
  _sim_speed =
      constrain_float(1.0f + 0.8f * sinf(2.0f * M_PI * t / 30.0f), 0.1f, 2.0f);
}

// =============================================================
// CHAN DOAN — canh bao lap lai moi 5s (thong bao luc boot de bi mat vi
// GCS chua ket noi) khi cam bien luu luong khong the hoat dong.
// =============================================================
void AP_ShoesAgtech::_flow_diag_check(void) {
  if (_simulation.get() > 0) {
    return;
  }
  const uint32_t now = AP_HAL::millis();
  if (now - _diag_warn_ms < 5000U) {
    return;
  }

  if (!_init_done) {
    _diag_warn_ms = now;
    gcs().send_text(MAV_SEVERITY_WARNING,
                    "SA: flow sensor not initialised - reboot required");
    return;
  }
  if (!_pin_gpio_ok) {
    _diag_warn_ms = now;
    uint8_t servo_ch = 0;
    if (hal.gpio->pin_to_servo_channel(_flow_pin_used, servo_ch)) {
      gcs().send_text(MAV_SEVERITY_WARNING,
                      "SA: pin %u not GPIO - set SERVO%u_FUNCTION=-1 + reboot",
                      (unsigned)_flow_pin_used, (unsigned)(servo_ch + 1));
    } else {
      gcs().send_text(MAV_SEVERITY_WARNING,
                      "SA: pin %u is not a valid GPIO on this board",
                      (unsigned)_flow_pin_used);
    }
    return;
  }
  if (!_irq_attached) {
    _diag_warn_ms = now;
    gcs().send_text(MAV_SEVERITY_WARNING, "SA: flow pin %u IRQ attach failed",
                    (unsigned)_flow_pin_used);
  }
}
