// MINUET GOVERNOR MODULE
//
// Pipeline:
//  1) Read raw sensor states
//  2) Massage & bundle: clamp/null handling (+ optional low-pass)
//  3) Controllers: Thermal, CO2, RH
//  4) Combine determinations
//  5) Apply inhibiting overrides
//  6) Return result
#pragma once

#include <cstdint>
#include <algorithm>
#include <cmath>

#include "core.h"
#include "esphome/core/log.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

namespace minuet {
namespace governor {

// -----------------------------------------------------------------------------
// Environmental Sensor handles (externally wired by YAML)
// -----------------------------------------------------------------------------

// Indoor

// Ambient temperature in Celsius.
inline esphome::sensor::Sensor* indoor_ambient_temperature_sensor{nullptr};
// Relative humidity in percent.
inline esphome::sensor::Sensor* indoor_relative_humidity_sensor{nullptr};
// Carbon dioxide concentration in ppm.
inline esphome::sensor::Sensor* indoor_co2_sensor{nullptr};
// Air quality index.
inline esphome::sensor::Sensor* indoor_aqi_sensor{nullptr};

//Outdoor

// Ambient temperature in Celsius.
inline esphome::sensor::Sensor* outdoor_ambient_temperature_sensor{nullptr};
// Ambient relative humidity in percent.
inline esphome::sensor::Sensor* outdoor_relative_humidity_sensor{nullptr};
// Ambient air quality index.
inline esphome::sensor::Sensor* outdoor_aqi_sensor{nullptr};

// -----------------------------------------------------------------------------
// Tunables
// -----------------------------------------------------------------------------

// Thermal mapping
static constexpr float kOutsideMarginC = 0.5f;   // °C don't cool below Tout + margin
static constexpr float kSpanAutoC  = 5.0f;       // °C span for AUTO to reach full scale
static constexpr float kSpanQuietC = 5.0f;       // °C span for QUIET to reach full scale
static constexpr float kGammaAuto  = 1.0f;       // Auto - Linear Ramp-up
static constexpr float kGammaQuiet = 2.5f;       // Quiet - Half-exponential ramp-up
// Hystersis handled by thermostat component.

// CO2 controller mapping
static constexpr bool  kEnableCO2Control    = true;
static constexpr float kCO2TargetPPM        = 700.0f;
static constexpr float kCO2DeadbandPPM      = 75.0f;
static constexpr float kCO2SpanPPM          = 500.0f;
static constexpr float kCO2Gamma            = 1.25f;

// RH controller mapping
static constexpr bool  kEnableRHControl       = true;
static constexpr float kRHTargetPct           = 60.0f;   // %
static constexpr float kRHDeadbandPct         = 5.0f;    // %
static constexpr float kRHGamma               = 1.0f;    // linear default
static constexpr float kRHSpanLoPct           = 60.0f;   // %
static constexpr float kRHSpanHiPct           = 100.0f;  // %
static constexpr float kRHOutsideMarginPct    = 5.0f;    // don't evacuate if RHo >= RHi + margin

// Optional sensor low-pass (1.0 = disabled / passthrough)
static constexpr float kAlphaTempLPF = 1.0f;
static constexpr float kAlphaRHLPF   = 1.0f;
static constexpr float kAlphaCO2LPF  = 1.0f;

// Levels
static constexpr int   kMaxLevel     = 10;  // global max discrete level
static constexpr int   kMinOnLevel   = 1;   // minimum running level
static constexpr int   kMaxLevelQuiet = 6;
static_assert(kMaxLevelQuiet >= kMinOnLevel && kMaxLevelQuiet <= kMaxLevel,
              "kMaxLevelQuiet must be in [kMinOnLevel, kMaxLevel]");

// Runtime toggles (HA switches sync these at boot & on change)
inline bool g_enable_co2_control = kEnableCO2Control;
inline bool g_enable_rh_control  = kEnableRHControl;

// -----------------------------------------------------------------------------
// Types
// -----------------------------------------------------------------------------
struct ControlInput {
  float ambient_temperature;   // °C (indoor)
  float target_temperature;    // °C (setpoint)
  ClimateAction action;        // thermostat action
  ClimateFanMode fan_mode;     // fan mode
  LidMode lid_mode;            // requested lid mode
};

// Active-controller reporting
enum class ActiveController : uint8_t { OFF = 0, THERMAL = 1, CO2 = 2, RH = 3 };

inline const char* active_controller_to_str(ActiveController c) {
  switch (c) {
    case ActiveController::THERMAL: return "Thermal";
    case ActiveController::CO2:     return "CO2";
    case ActiveController::RH:      return "RH";
    case ActiveController::OFF:
    default:                        return "Off";
  }
}

struct ControlOutput {
  int  fan_speed{0};           // 0–10
  bool lid_open{false};        // lid state
  ActiveController active_controller{ActiveController::OFF}; // intent (pre-override)
};

// Snapshot of raw reads (pre-massage)
struct SensorSample {
  bool has_Tin{false}, has_Tout{false}, has_RHi{false}, has_RHo{false}, has_CO2{false};
  float Tin{NAN}, Tout{NAN}, RHi{NAN}, RHo{NAN}, CO2{NAN};
};

// Sanitized/filtered bundle every controller consumes
struct SensorBundle {
  bool has_Tin{false}, has_Tout{false}, has_RHi{false}, has_RHo{false}, has_CO2{false};
  float Tin{0}, Tout{0}, RHi{0}, RHo{0}, CO2{0};       // clamped
  float Tin_f{0}, RHi_f{0}, CO2_f{0};                  // filtered (LPF), if enabled
};

// Controller-to-arbiter result
struct Determination {
  int  level{0};           // requested fan level 0..10
  bool lid_request{false}; // airflow path desired
  bool active{false};      // after internal hysteresis/gating
};

// Mutable state (hysteresis + LPF memory)
struct GovernorState {
  // Hysteresis latches
  bool co2_active{false};
  bool rh_active{false};

  // LPF memory
  bool lpf_init_Tin{false}, lpf_init_RHi{false}, lpf_init_CO2{false};
  float Tin_prev{0}, RHi_prev{0}, CO2_prev{0};
};

// -----------------------------------------------------------------------------
// Module state and helpers
// -----------------------------------------------------------------------------
// Global persistent state
inline GovernorState g_state{};

inline bool sensor_valid(const esphome::sensor::Sensor* s) {
  return s && s->has_state() && std::isfinite(s->state);
}

template <typename T>
inline T clampf(T x, T lo, T hi) { return std::min(std::max(x, lo), hi); }

inline float lpf_step(float x, float prev, float alpha) {
  // alpha in [0,1], alpha=1 -> passthrough
  return alpha * x + (1.0f - alpha) * prev;
}

// Reset API clears hysteresis & LPF state
inline void reset() {
  g_state = GovernorState{};
}

// -----------------------------------------------------------------------------
// (1) Read sensors -> SensorSample
// -----------------------------------------------------------------------------
inline SensorSample read_sensors() {
  SensorSample s{};
  if (sensor_valid(indoor_ambient_temperature_sensor)) {
    s.has_Tin = true; s.Tin = indoor_ambient_temperature_sensor->state;
  }
  if (sensor_valid(outdoor_ambient_temperature_sensor)) {
    s.has_Tout = true; s.Tout = outdoor_ambient_temperature_sensor->state;
  }
  if (sensor_valid(indoor_relative_humidity_sensor)) {
    s.has_RHi = true; s.RHi = indoor_relative_humidity_sensor->state;
  }
  if (sensor_valid(outdoor_relative_humidity_sensor)) {
    s.has_RHo = true; s.RHo = outdoor_relative_humidity_sensor->state;
  }
  if (sensor_valid(indoor_co2_sensor)) {
    s.has_CO2 = true; s.CO2 = indoor_co2_sensor->state;
  }
  return s;
}

// -----------------------------------------------------------------------------
// (2) Massage + bundle (clamp + optional LPF)
// -----------------------------------------------------------------------------
inline SensorBundle massage_bundle(const SensorSample& s) {
  SensorBundle b{};

  // Clamp ranges (adjust to your sensor specs if needed)
  // Temp: [-40, 85] °C, RH: [0,100] %, CO2: [0,5000] ppm
  if (s.has_Tin)  { b.has_Tin  = true; b.Tin  = clampf(s.Tin,  -40.0f, 85.0f); }
  if (s.has_Tout) { b.has_Tout = true; b.Tout = clampf(s.Tout, -40.0f, 85.0f); }
  if (s.has_RHi)  { b.has_RHi  = true; b.RHi  = clampf(s.RHi,    0.0f,100.0f); }
  if (s.has_RHo)  { b.has_RHo  = true; b.RHo  = clampf(s.RHo,    0.0f,100.0f); }
  if (s.has_CO2)  { b.has_CO2  = true; b.CO2  = clampf(s.CO2,    0.0f,5000.0f); }

  // LPF (disabled by default via alpha=1.0)
  if (b.has_Tin) {
    if (!g_state.lpf_init_Tin) { g_state.Tin_prev = b.Tin; g_state.lpf_init_Tin = true; }
    b.Tin_f = lpf_step(b.Tin, g_state.Tin_prev, kAlphaTempLPF);
    g_state.Tin_prev = b.Tin_f;
  }
  if (b.has_RHi) {
    if (!g_state.lpf_init_RHi) { g_state.RHi_prev = b.RHi; g_state.lpf_init_RHi = true; }
    b.RHi_f = lpf_step(b.RHi, g_state.RHi_prev, kAlphaRHLPF);
    g_state.RHi_prev = b.RHi_f;
  }
  if (b.has_CO2) {
    if (!g_state.lpf_init_CO2) { g_state.CO2_prev = b.CO2; g_state.lpf_init_CO2 = true; }
    b.CO2_f = lpf_step(b.CO2, g_state.CO2_prev, kAlphaCO2LPF);
    g_state.CO2_prev = b.CO2_f;
  }

  // Diagnostics: one-line snapshot of bundle
  ESP_LOGD("governor",
           "Bundle: Tin=%s Tout=%s RHi=%s RHo=%s CO2=%s",
           b.has_Tin  ? esphome::str_sprintf("%.2f/%.2f", b.Tin, b.Tin_f).c_str() : "n/a",
           b.has_Tout ? esphome::str_sprintf("%.2f", b.Tout).c_str()               : "n/a",
           b.has_RHi  ? esphome::str_sprintf("%.1f/%.1f", b.RHi, b.RHi_f).c_str()  : "n/a",
           b.has_RHo  ? esphome::str_sprintf("%.1f", b.RHo).c_str()                : "n/a",
           b.has_CO2  ? esphome::str_sprintf("%.0f/%.0f", b.CO2, b.CO2_f).c_str()  : "n/a");

  return b;
}

// -----------------------------------------------------------------------------
// (3) Controllers
// -----------------------------------------------------------------------------
inline Determination determine_thermal(const ControlInput& input, const SensorBundle& b) {
  Determination d{};
  if (input.action != ClimateAction::CLIMATE_ACTION_COOLING) return d;
  if (!b.has_Tin) return d;

  const float Tin  = b.Tin_f;     // filtered Tin
  const float Tset = input.target_temperature;

  if (!std::isfinite(Tset)) {
    ESP_LOGW("governor", "Thermal: invalid Tset=%.2f", Tset);
    return d;
  }

  float Tout = 0.0f;
  const bool has_out = b.has_Tout ? (Tout = b.Tout, true) : false;

  // Don't cool below outdoor + margin
  const float target_floor = has_out ? std::max(Tset, Tout + kOutsideMarginC) : Tset;
  const float error        = Tin - target_floor;

  const bool  quiet = (input.fan_mode == ClimateFanMode::CLIMATE_FAN_QUIET);
  const float span  = quiet ? kSpanQuietC : kSpanAutoC;
  const float gamma = quiet ? kGammaQuiet  : kGammaAuto;

  const float drive   = clampf(error / span, 0.0f, 1.0f);
  const float level_f = static_cast<float>(kMaxLevel) * std::pow(drive, gamma);
  d.level             = static_cast<int>(std::ceil(level_f));
  d.active            = (d.level > 0);
  d.lid_request       = d.active;

  ESP_LOGD("governor",
           "Thermal: Tin=%.2f Tout=%s Tset=%.2f target_floor=%.2f error=%.2f level=%d",
           Tin,
           has_out ? esphome::str_sprintf("%.2f", Tout).c_str() : "n/a",
           Tset, target_floor, error, d.level);
  return d;
}

inline Determination determine_co2(const SensorBundle& b, GovernorState& st) {
  Determination d{};
  if (!(kEnableCO2Control && g_enable_co2_control) || !b.has_CO2) return d;

  const float co2 = b.CO2_f;
  const float target_hi = kCO2TargetPPM + kCO2DeadbandPPM;
  const float target_lo = kCO2TargetPPM - kCO2DeadbandPPM;

  // Hysteresis transitions
  if (!st.co2_active && co2 >= target_hi) st.co2_active = true;
  else if (st.co2_active && co2 <= target_lo) st.co2_active = false;

  if (st.co2_active) {
    if (co2 <= kCO2TargetPPM) {
      d.level = kMinOnLevel;
    } else {
      const float drive   = clampf((co2 - kCO2TargetPPM) / kCO2SpanPPM, 0.0f, 1.0f);
      const float level_f = static_cast<float>(kMaxLevel) * std::pow(drive, kCO2Gamma);
      d.level             = std::max(kMinOnLevel, static_cast<int>(std::ceil(level_f)));
    }
    d.active      = (d.level > 0);
    d.lid_request = d.active;
  }

  ESP_LOGD("governor",
           "CO2: co2=%.0f target=%.0f deadband=%.0f active=%d level=%d",
           co2, kCO2TargetPPM, kCO2DeadbandPPM, st.co2_active, d.level);
  return d;
}

inline Determination determine_rh(const SensorBundle& b, GovernorState& st) {
  Determination d{};
  // Require both indoor and outdoor RH sensors
  const bool rh_inputs_ok = (kEnableRHControl && g_enable_rh_control && b.has_RHi && b.has_RHo);
  if (!rh_inputs_ok) return d;

  const float RHi = b.RHi_f;  // use filtered indoor RH
  const float RHo = b.RHo;

  // Block evacuation if outdoor humidity >= indoor + margin
  const bool outdoor_block = (RHo >= (RHi + kRHOutsideMarginPct));

  const float target_hi = kRHTargetPct + kRHDeadbandPct;
  const float target_lo = kRHTargetPct - kRHDeadbandPct;

  // Hysteresis transitions (respect outdoor gating)
  if (!st.rh_active && !outdoor_block && (RHi >= target_hi)) {
    st.rh_active = true;
  } else if (st.rh_active && (RHi <= target_lo || outdoor_block)) {
    st.rh_active = false;
  }

  if (st.rh_active) {
    if (RHi <= kRHTargetPct) {
      d.level = kMinOnLevel;
    } else {
      const float span   = (kRHSpanHiPct - kRHSpanLoPct); // 40% per spec
      const float drive  = clampf((RHi - kRHTargetPct) / span, 0.0f, 1.0f);
      const float level_f= static_cast<float>(kMaxLevel) * std::pow(drive, kRHGamma);
      d.level            = std::max(kMinOnLevel, static_cast<int>(std::ceil(level_f)));
    }
    d.active      = (d.level > 0);
    d.lid_request = d.active;
  }

  ESP_LOGD("governor",
           "RH: RHi=%.1f RHo=%.1f target=%.1f deadband=%.1f block=%d active=%d level=%d",
           RHi, RHo, kRHTargetPct, kRHDeadbandPct, outdoor_block, st.rh_active, d.level);
  return d;
}

// -----------------------------------------------------------------------------
// (4) Combine determinations
// -----------------------------------------------------------------------------
inline void combine(const Determination& t,
                    const Determination& c,
                    const Determination& h,
                    int& level_raw,
                    bool& any_controller_active,
                    bool& any_lid_request,
                    ActiveController& pre_override_active) {
  level_raw = std::max({t.level, c.level, h.level});
  any_controller_active = (t.active || c.active || h.active);
  any_lid_request       = (t.lid_request || c.lid_request || h.lid_request);

  // Deterministic tie-breaker: Thermal > CO2 > RH
  pre_override_active = ActiveController::OFF;
  int best = 0;
  if (t.level > best) { best = t.level; pre_override_active = ActiveController::THERMAL; }
  if (c.level > best) { best = c.level; pre_override_active = ActiveController::CO2; }
  if (h.level > best) { best = h.level; pre_override_active = ActiveController::RH; }
}

// -----------------------------------------------------------------------------
// (5) Apply inhibiting overrides (fan mode + min-speed rule + lid overrides)
// -----------------------------------------------------------------------------
inline void apply_overrides(const ControlInput& input,
                            int level_raw,
                            bool any_controller_active,
                            bool any_lid_request,
                            ControlOutput& output) {
  // Publish active-controller intent with first-publish debounce
  static ActiveController last_active = ActiveController::OFF;
  static bool published_once = false;
  if (!published_once || output.active_controller != last_active) {
    last_active = output.active_controller;
    published_once = true;
    #ifdef ESPHOME_VERSION_CODE
    id(minuet_active_controller).publish_state(active_controller_to_str(last_active));
    #endif
  }

  const bool cooling_active   = (input.action == ClimateAction::CLIMATE_ACTION_COOLING);
  const bool should_force_min = cooling_active || any_controller_active;

  int level = level_raw;
  switch (input.fan_mode) {
    case ClimateFanMode::CLIMATE_FAN_OFF:
      level = 0; // manual OFF wins
      break;

    case ClimateFanMode::CLIMATE_FAN_LOW:
      // respect min run if needed
      level = should_force_min ? std::max(level_raw, kMinOnLevel) : 0;
      break;

    case ClimateFanMode::CLIMATE_FAN_QUIET: {
      const int cap = std::min(kMaxLevelQuiet, kMaxLevel);
      if (should_force_min) {
        level = std::clamp(std::max(level_raw, kMinOnLevel), kMinOnLevel, cap);
      } else {
        level = 0;
      }
      break;
    }

    case ClimateFanMode::CLIMATE_FAN_AUTO:
    default:
      if (should_force_min) {
        level = std::clamp(std::max(level_raw, kMinOnLevel), kMinOnLevel, kMaxLevel);
      } else {
        level = 0;
      }
      break;
  }

  // Final clamp to correct cap for the current mode
  const int global_cap = (input.fan_mode == ClimateFanMode::CLIMATE_FAN_QUIET)
                         ? std::min(kMaxLevelQuiet, kMaxLevel)
                         : kMaxLevel;
  level = std::clamp(level, 0, global_cap);

  // Default lid policy: open if any controller requested airflow
  output.lid_open  = any_lid_request;
  output.fan_speed = level;

  // Explicit lid-mode overrides
  switch (input.lid_mode) {
    case LidMode::OPEN:   output.lid_open = true;  break;
    case LidMode::CLOSED: output.lid_open = false; break;
    case LidMode::AUTO:
    default: break;
  }
}

// -----------------------------------------------------------------------------
// (6) Main control entry
// -----------------------------------------------------------------------------
[[nodiscard]] inline ControlOutput update(const ControlInput& input) {
  ControlOutput output{};

  // (1) Read
  SensorSample sample = read_sensors();

  // (2) Massage & bundle
  SensorBundle bundle = massage_bundle(sample);

  // (3) Controllers
  Determination det_thermal = determine_thermal(input, bundle);
  Determination det_co2     = determine_co2(bundle, g_state);
  Determination det_rh      = determine_rh(bundle, g_state);

  // (4) Combine
  int level_raw = 0;
  bool any_controller_active = false;
  bool any_lid_request = false;
  ActiveController pre_override_active = ActiveController::OFF;
  combine(det_thermal, det_co2, det_rh,
          level_raw, any_controller_active, any_lid_request, pre_override_active);
  output.active_controller = pre_override_active;

  // (5) Overrides -> (6) Output
  apply_overrides(input, level_raw, any_controller_active, any_lid_request, output);
  return output;
}

}  // namespace governor
}  // namespace minuet
