// MINUET GOVERNOR MODULE
//
// Determines the fan speed and lid state under automatic thermostat control
// based on the thermostat state, user preferences, and environmental sensors.
#pragma once

#include <cstdint>
#include <algorithm>  // for std::clamp
#include <cmath>      // for std::pow, std::lround, std::isfinite

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

// Outdoor relative humidity in percent.
esphome::sensor::Sensor* outdoor_relative_humidity_sensor{nullptr};

// Outdoor air quality index.
esphome::sensor::Sensor* outdoor_aqi_sensor{nullptr};

// -----------------------------------------------------------------------------
// Tunable constants (no deadband/hysteresis here; thermostat handles that)
// -----------------------------------------------------------------------------
static constexpr float kOutsideMarginC        = 0.5f;  // °C don't cool below Tout + margin
static constexpr float kSpanAutoC             = 5.0f;  // °C error span mapped to full-scale AUTO
static constexpr float kSpanQuietC            = 7.5f;  // °C error span mapped to full-scale QUIET
static constexpr float kGammaAuto             = 1.0f;  // linear in AUTO
static constexpr float kGammaQuiet            = 3.75f; // quieter ramp in QUIET
static constexpr int   kMaxLevel              = 10;    // global max discrete level
static constexpr int   kMaxAuto               = 10;    // mode cap for AUTO
static constexpr int   kMaxQuiet              = 10;    // mode cap for QUIET

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
inline void reset() {
}

// -----------------------------------------------------------------------------
// Main control logic (stateless mapping + outdoor safeguard)
// -----------------------------------------------------------------------------
inline ControlOutput update(const ControlInput& input) {
  ControlOutput output{};

  // Only run fan logic when thermostat says we are cooling.
  if (input.action == ClimateAction::CLIMATE_ACTION_COOLING) {
    const float Tin  = input.ambient_temperature;
    const float Tset = input.target_temperature;

    // Guard against NaNs from sensors
    if (!std::isfinite(Tin) || !std::isfinite(Tset)) {
      ESP_LOGW("governor", "Invalid temperature(s): Tin=%.2f Tset=%.2f", Tin, Tset);
      output.fan_speed = 0;
      output.lid_open  = false;
      return output;
    }

    // Outdoor temperature if available
    float Tout = 0.0f;
    bool  has_out = false;
    if (outdoor_ambient_temperature_sensor && outdoor_ambient_temperature_sensor->has_state()) {
      Tout = outdoor_ambient_temperature_sensor->state;
      has_out = std::isfinite(Tout);
    }

    // Don't cool below outdoor + margin (ventilation sanity)
    const float target_floor = has_out ? std::max(Tset, Tout + kOutsideMarginC) : Tset;
    const float error = Tin - target_floor;

    ESP_LOGD("governor", "Tin=%.2f Tout=%s Tset=%.2f target_floor=%.2f error=%.2f",
             Tin,
             has_out ? esphome::str_sprintf("%.2f", Tout).c_str() : "n/a",
             Tset, target_floor, error);

    // Shape error -> [0..1] drive, then to [0..kMaxLevel] speed with gamma curve
    const bool  quiet  = (input.fan_mode == ClimateFanMode::CLIMATE_FAN_QUIET);
    const float span   = quiet ? kSpanQuietC : kSpanAutoC;
    const float gamma  = quiet ? kGammaQuiet  : kGammaAuto;

    float drive   = std::clamp(error / span, 0.0f, 1.0f);
    float level_f = static_cast<float>(kMaxLevel) * std::pow(drive, gamma);
    int   level   = static_cast<int>(std::lround(level_f));

    // Fan mode overrides (minimum speed of 1 in QUIET/AUTO as requested)
    switch (input.fan_mode) {
      case ClimateFanMode::CLIMATE_FAN_OFF:   level = 0; break;
      case ClimateFanMode::CLIMATE_FAN_LOW:   level = 1; break;
      case ClimateFanMode::CLIMATE_FAN_QUIET: level = std::clamp(level, 1, kMaxQuiet); break;
      case ClimateFanMode::CLIMATE_FAN_AUTO:
      default:                                level = std::clamp(level, 1, kMaxAuto);  break;
    }

    // Final clamp to supported range
    level = std::clamp(level, 0, kMaxLevel);

    output.fan_speed = level;
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
