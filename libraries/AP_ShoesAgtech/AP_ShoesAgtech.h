#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

// =============================================================
// AP_ShoesAgtech (S30) — clone tu ban S16, CHI giu Module 1 (Bom):
// cam bien luu luong YF-S402B + dieu khien bom (passthrough / PID).
// Da bo toan bo phan theo ao (SA_POND_IDX), Module 2 (pH) va Module 3
// (cho an) — khong luu chi so ao / du lieu ao len the SD.
// Chi so tham so (index) giu NGUYEN nhu S16 de file .param tuong thich.
// =============================================================

class AP_ShoesAgtech_FlowParams {
public:
  static const AP_Param::GroupInfo var_info[];

  AP_Float cal_factor;    // SA_CAL_FAC
  AP_Float ema_alpha;     // SA_FLOW_EMA_AL
  AP_Int8  flow_log_enable; // SA_FLOW_LOG
  AP_Int8  rc_chan;       // SA_RC_CHAN
  AP_Int8  rc_pump;       // SA_RC_PUMP
  AP_Int8  pump_chan;     // SA_PUMP_CHAN
  AP_Float flow_setpoint; // SA_FLOW_SP
  AP_Float pid_p;         // SA_FLOW_PID_P
  AP_Float pid_i;         // SA_FLOW_PID_I
  AP_Float pid_lpf;       // SA_FLOW_PID_LPF
  AP_Int16 flow_log_ms;   // SA_FLOW_LOG_MS
  AP_Int16 flow_pin;      // SA_FLOW_PIN
  AP_Float tank_vol;      // SA_TANK_VOL
  AP_Int8  flow_mode;     // SA_FLOW_MODE
  AP_Float mix_std;       // SA_FLOW_MIX_STD
  AP_Float mix_cnt;       // SA_FLOW_MIX_CNT
  AP_Float flow_vel;      // SA_FLOW_VEL
};

// Module 3 rut gon — chi dia rai ly tam (khong truc vit)
class AP_ShoesAgtech_DiscParams {
public:
  static const AP_Param::GroupInfo var_info[];

  AP_Int8  dos_rc;      // SA_DOS_RC     kenh RC bat/tat dia rai
  AP_Int8  log_enable;  // SA_DOS_LOG
  AP_Int16 log_ms;      // SA_DOS_LOG_MS
  AP_Int8  disc_chan;   // SA_DISC_CHAN  kenh servo/ESC dia rai, 0=tat
  AP_Int8  disc_pct;    // SA_DISC_PCT   % toc do dia khi chay
  AP_Int8  disc_rev;    // SA_DISC_REV   0=StandardESC, 1=BidirForward, 2=BidirReverse
  AP_Int16 disc_min;    // SA_DISC_MIN   PWM 0% khi REV=0
};

class AP_ShoesAgtech {
public:
  AP_ShoesAgtech();

  CLASS_NO_COPY(AP_ShoesAgtech);

  void init(void);
  void update(void);

  // Toc do DAT cho mission (WP_SPEED, da cap nhat qua DO_CHANGE_SPEED/GCS
  // SET_SPEED) — Rover.cpp goi moi chu ky TRUOC update(). Dung lam toc do
  // tham chieu cho cong thuc FLOW_MODE=1 (_compute_visin_target()).
  void set_target_speed(float speed) { _target_speed = speed; }

  bool     is_enabled(void)   const { return _enable_flag.get() > 0; }
  int16_t  get_robot_id(void) const { return _robot_id.get(); }

  // ---- Flow sensor getters — YF-S402B, day hoat dong 0.3–6 L/min ----
  float    get_flow_rate_lmin(void) const { return _flow_rate_filtered; }
  float    get_flow_rate_avg(void)  const { return _flow_rate_avg; }

  // Spray control getters (for logging)
  uint8_t  get_spray_mode(void)  const { return _spray_mode; }
  uint16_t get_pump_pwm(void)    const { return _pump_pwm; }
  float    get_flow_target(void) const { return _flow_target; }

  // ---- Spreader disc getters ----
  uint16_t get_disc_pwm(void)     const { return _disc_pwm; }
  bool     get_disc_running(void) const { return _disc_running; }

  static const AP_Param::GroupInfo var_info[];
  static void irq_handler(void);

private:
  // ================================================================
  // PARAMETERS
  // ================================================================
  AP_Int8  _enable_flag;    // SA_ENABLE
  AP_Int16 _robot_id;       // SA_ROBOT_ID  dinh danh robot (0-999), khong dung noi bo
  AP_Int8  _simulation;     // SA_SIM  0=cam bien that, 1=du lieu gia lap
  AP_Int8  _spd_start_pct;  // SA_SPD_START  % toc do dat toi thieu de bat dau bom (FLOW_MODE=1)

  AP_ShoesAgtech_FlowParams _flow_params; // Module 1 — Bom
  AP_ShoesAgtech_DiscParams _disc_params; // Module 3 rut gon — Dia rai

  // ================================================================
  // STATE
  // ================================================================

  // ---- flow sensor state — YF-S402B ----
  uint32_t _last_timestamp_ms;
  uint32_t _last_pulse_snapshot;
  uint32_t _last_log_ms;
  float    _flow_rate_filtered;
  float    _flow_rate_avg;
  bool     _is_initialized;

  static const uint8_t WINDOW_SIZE = 10;
  float    _sample_buffer[WINDOW_SIZE];
  uint8_t  _buffer_index;
  float    _buffer_sum;
  uint8_t  _samples_count;

  static volatile uint32_t _pulse_count;

  // ---- spray control state ----
  uint8_t  _spray_mode;         // 0=PASSTHROUGH 1=FLOW_PID(MIX_STD) 2=FLOW_PID(MIX_CNT)
  uint16_t _pump_pwm;
  float    _flow_target;
  // Setpoint thuc te dua vao PID — tang dan (ramp) tu 0 len _flow_target
  // de tranh bom giat/tran luc moi moi.
  float    _flow_ramp_val;
  float    _pid_integral;
  float    _pid_output_lpf;
  uint32_t _pid_last_ms;

  // ---- pump config check state ----
  int8_t   _last_pump_chan;
  int32_t  _last_pump_func_val;
  bool     _pump_config_ok;

  // ---- mission distance cache + tank monitor state ----
  float    _mission_dist_m;
  uint16_t _mission_ncmds;
  bool     _tank_empty_detected;
  uint32_t _tank_empty_ms;
  // "no mission" (FLOW_MODE=1) — chi canh bao 1 lan moi phien ARM
  bool     _no_mission_warned;
  // q1 (FLOW_MODE=1) ngoai dai luu luong that bom dat duoc (0.8-1.3 L/min)
  bool     _flow_out_of_range;
  float    _target_speed;

  // ---- SIMULATION state ----
  float    _sim_speed;

  // ---- spreader disc state ----
  uint16_t _disc_pwm;
  bool     _disc_running;
  bool     _disc_was_on;
  bool     _disc_config_ok;
  bool     _disc_was_ok;
  uint32_t _disc_warn_ms;
  bool     _disc_rc_seen_off;   // SA_DOS_RC da ve OFF it nhat 1 lan tu luc boot
  uint32_t _disc_rc_check_ms;
  uint32_t _disc_last_log_ms;

  // ---- CHAN DOAN cam bien luu luong ----
  bool     _init_done;     // init() da chay (SA_ENABLE=1 luc boot)
  bool     _pin_gpio_ok;   // SA_FLOW_PIN da duoc bat lam GPIO (SERVOx_FUNCTION=-1)
  bool     _irq_attached;  // gan ngat thanh cong
  uint8_t  _flow_pin_used; // chan thuc te da gan ngat luc boot
  uint32_t _diag_warn_ms;

  // ================================================================
  // PRIVATE METHODS
  // ================================================================
  void     _update_flow(void);
  void     _update_spray_mode(void);
  void     _check_pump_config(void);
  uint16_t _run_flow_pid(float target_lmin, float dt);
  void     _write_pump_pwm(uint16_t pwm);
  float    _get_spray_speed(void);
  float    _get_dosing_ref_speed(void);
  float    _compute_visin_target(float r);
  float    _get_mission_dist(void);
  float    _speed_min_start(void);

  void     _run_simulation(void);
  void     _flow_diag_check(void);

  void     _check_disc_config(void);
  void     _update_disc(void);
};
