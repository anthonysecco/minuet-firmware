// MINUET GOVERNOR MODULE
//
// Determines the fan speed and lid state under automatic thermostat control
// based on the thermostat state, user preferences, and environmental sensors.
#pragma once

#include <cstdint>
#include <algorithm>  // for std::clamp
#include <cmath>      // for std::pow, std::lround

#include "core.h"
#include "esphome/core/log.h"
#include "esphome/components/sensor/sensor.h"

namespace minuet {
namespace governor {

// ENVIRONMENTAL SENSORS

// Indoor ambient temperature in Celsius.
esphome::sensor::Sensor* indoor_ambient_temperature_sensor{nullptr};

// Indoor relative humidity in percent.
esphome::sensor::Sensor* indoor_relative_humidity_sensor{nullptr};

// Indoor carbon dioxide concentration in ppm.
esphome::sensor::Sensor* indoor_co2_sensor{nullptr};

// Indoor air quality index.
esphome::sensor::Sensor* indoor_aqi_sensor{nullptr};

// Outdoor ambient temperature in Celsius.
esphome::sensor::Sensor* outdoor_ambient_temperature_sensor{nullptr};

// Outdoor relative humidify in percent.
esphome::sensor::Sensor* outdoor_relative_humidity_sensor{nullptr};

// Outdoor air quality index.
esphome::sensor::Sensor* outdoor_aqi_sensor{nullptr};

// -----------------------------------------------------------------------------
// Tunable constants
// -----------------------------------------------------------------------------
static constexpr float kDeadbandC             = 0.3f;
static constexpr float kOutsideMarginC        = 0.5f;
static constexpr float kSpanAutoC             = 5.0f;
static constexpr float kSpanQuietC            = 7.5f;
static constexpr float kGammaAuto             = 1.0f;
static constexpr float kGammaQuiet            = 3.75f; // 2.5 to optimized for energy, 5.0 to optimize for quiet, 3.75 is a good middle ground
static constexpr int   kMaxVentWhenHotOutside = 3;
static constexpr int   kMaxAuto               = 10;
static constexpr int   kMaxQuiet              = 10;
static constexpr float kEMAAlpha              = 0.4f;
static constexpr int   kMinPersistTicks       = 2;
static constexpr int   kMaxStepPerTick        = 1;

// Internal smoothing/hysteresis state
static bool  s_ema_initialized = false;
static float s_tin_ema         = 0.0f;
static int   s_current_speed   = 0;
static int   s_proposed_speed  = 0;
static int   s_persist_count   = 0;

// -----------------------------------------------------------------------------
// Input / Output structures
// -----------------------------------------------------------------------------
struct ControlInput {
  float ambient_temperature;   // °C (indoor)
  float target_temperature;    // °C (setpoint)
  ClimateAction action;        // thermostat action
  ClimateFanMode fan_mode;     // fan mode
  LidMode lid_mode;            // requested lid mode
};

struct ControlOutput {
  int  fan_speed{0};           // 0–10
  bool lid_open{false};        // lid state
};

// -----------------------------------------------------------------------------
// Reset internal state
// -----------------------------------------------------------------------------
void reset() {
  s_ema_initialized = false;
  s_tin_ema         = 0.0f;
  s_current_speed   = 0;
  s_proposed_speed  = 0;
  s_persist_count   = 0;
}

// -----------------------------------------------------------------------------
// Main control logic
// -----------------------------------------------------------------------------
ControlOutput update(ControlInput input) {
  ControlOutput output{};

  if (input.action == ClimateAction::CLIMATE_ACTION_COOLING) {
    // Exponential moving average on indoor temperature
    const float tin_raw = input.ambient_temperature;
    if (!s_ema_initialized) {
      s_tin_ema = tin_raw;
      s_ema_initialized = true;
    } else {
      s_tin_ema = kEMAAlpha * tin_raw + (1.0f - kEMAAlpha) * s_tin_ema;
    }

    const float Tin  = s_tin_ema;
    const float Tset = input.target_temperature;

    // Outdoor temperature if available
    float Tout = 0.0f;
    bool  has_out = false;
    if (outdoor_ambient_temperature_sensor && outdoor_ambient_temperature_sensor->has_state()) {
      Tout = outdoor_ambient_temperature_sensor->state;
      has_out = true;
    }

    // Adjust target to not cool below outdoor + margin
    const float target_floor = has_out ? std::max(Tset, Tout + kOutsideMarginC) : Tset;
    const float e = Tin - target_floor;

    ESP_LOGD("governor", "Tin=%.2f Tout=%.2f Tset=%.2f target_floor=%.2f e=%.2f",
             Tin, Tout, Tset, target_floor, e);

    int level = 0;
    if (e > kDeadbandC) {
      const bool quiet  = (input.fan_mode == ClimateFanMode::CLIMATE_FAN_QUIET);
      const float span  = quiet ? kSpanQuietC : kSpanAutoC;
      const float gamma = quiet ? kGammaQuiet  : kGammaAuto;

      float drive   = std::clamp(e / span, 0.0f, 1.0f);
      float level_f = 10.0f * std::pow(drive, gamma);
      level = static_cast<int>(std::lround(level_f));

      if (has_out && Tin <= Tout + kOutsideMarginC)
        level = std::min(level, kMaxVentWhenHotOutside);
    }

    switch (input.fan_mode) {
      case ClimateFanMode::CLIMATE_FAN_OFF:   level = 0; break;
      case ClimateFanMode::CLIMATE_FAN_LOW:   level = 1; break;
      case ClimateFanMode::CLIMATE_FAN_QUIET: level = std::clamp(level, 1, kMaxQuiet); break;
      case ClimateFanMode::CLIMATE_FAN_AUTO:
      default:                                level = std::clamp(level, 1, kMaxAuto); break;
    }

    // Apply rate limiting and persistence
    s_proposed_speed = std::clamp(level, 0, 10);
    int next = s_current_speed;
    if (s_proposed_speed != s_current_speed) {
      if (++s_persist_count >= kMinPersistTicks) {
        int delta = s_proposed_speed - s_current_speed;
        delta = std::clamp(delta, -kMaxStepPerTick, kMaxStepPerTick);
        next = s_current_speed + delta;
        s_persist_count = 0;
      }
    } else {
      s_persist_count = 0;
    }
    s_current_speed = std::clamp(next, 0, 10);

    output.fan_speed = s_current_speed;
    output.lid_open  = true;
  } else {
    output.fan_speed = 0;
    output.lid_open  = false;
  }

  // Explicit lid mode overrides
  switch (input.lid_mode) {
    case LidMode::OPEN:   output.lid_open = true;  break;
    case LidMode::CLOSED: output.lid_open = false; break;
    case LidMode::AUTO:
    default: break; // already handled by climate action
  }

  return output;
}

}  // namespace governor
}  // namespace minuet
