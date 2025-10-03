// MINUET GOVERNOR MODULE
//
// Determines the fan speed and lid state under automatic thermostat control
// based on the thermostat state, user preferences, and environmental sensors.
#include <cstdint>

#include "esphome/core/log.h"
#include "esphome/components/sensor/sensor.h"

// ADDED: minimal headers for math/clamp
#include <algorithm>  // ADDED
#include <cmath>      // ADDED

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

// The lid behavior when the thermostat is enabled.
enum class LidMode : uint8_t {
  AUTO = 0,
  KEEP_OPEN = 1,
  KEEP_CLOSED = 2,
};

// Tunables (kept as constexpr for easy tweaking)
static constexpr float kDeadbandC             = 0.3f;   // ADDED
static constexpr float kOutsideMarginC        = 0.5f;   // ADDED
static constexpr float kSpanC                 = 5.0f;   // ADDED
static constexpr float kGammaAuto             = 1.0f;   // ADDED
static constexpr float kGammaQuiet            = 1.75f;   // ADDED
static constexpr int   kMaxVentWhenHotOutside = 3;      // ADDED
static constexpr int   kMaxAuto               = 10;     // ADDED
static constexpr int   kMaxQuiet              = 6;      // ADDED
static constexpr float kEMAAlpha              = 0.4f;   // ADDED
static constexpr int   kMinPersistTicks       = 2;      // ADDED
static constexpr int   kMaxStepPerTick        = 1;      // ADDED

// Small internal state for exponential moving average + hysteresis/rate limiting
static bool  s_ema_initialized = false;  // ADDED
static float s_tin_ema         = 0.0f;   // ADDED
static int   s_current_speed   = 0;      // ADDED
static int   s_proposed_speed  = 0;      // ADDED
static int   s_persist_count   = 0;      // ADDED

// The governor's control inputs.
struct ControlInput {
  // Ambient temperature in °C.  This is the value that the thermostat most recently
  // sampled from `indoor_ambient_temperature_sensor`.
  float ambient_temperature;
  
  // Target temperature in °C.  This is the thermostat's setpoint.
  float target_temperature;
  
  // The requested climate action considering the current temperature, target temperature,
  // and thermostat hysteresis parameters.
  // - `CLIMATE_ACTION_OFF`: no demand
  // - `CLIMATE_ACTION_COOLING`: demand for cooling
  ClimateAction action;
  
  // The requested fan behavior.
  // - `CLIMATE_FAN_OFF`: use passive ventilation
  // - `CLIMATE_FAN_LOW`: use the lowest fan speed only
  // - `CLIMATE_FAN_AUTO`: control the fan speed automatically
  // - `CLIMATE_FAN_QUIET`: control the fan speed automatically and optimize for quietness
  ClimateFanMode fan_mode;

  // The requested lid behavior.
  // - `AUTO`: open the lid when cooling, close the lid when off
  // - `KEEP_OPEN`: keep the lid open for passive ventilation
  // - `KEEP_CLOSED`: keep the lid closed for ceiling fan mode
  LidMode lid_mode;
};

// The governor's control outputs.
struct ControlOutput {
  // The requested fan speed from 0 (off) to 10 (maximum).
  // TODO: Consider making this floating point to allow for interpolation.
  int fan_speed{0};

  // True if the lid should be open.
  bool lid_open{false};
};

// Resets the governor's internal state.
// Called when the thermostat is enabled.
void reset() {
  // ADDED: clear internal smoothing + hysteresis state
  s_ema_initialized = false;  // ADDED
  s_tin_ema         = 0.0f;   // ADDED
  s_current_speed   = 0;      // ADDED
  s_proposed_speed  = 0;      // ADDED
  s_persist_count   = 0;      // ADDED
}

// Determines the next set of control outputs from the provided inputs and sensors.
// Called when the inputs change and periodically.
ControlOutput update(ControlInput input) {
  ControlOutput output{};
  if (input.action == CLIMATE_ACTION_COOLING) {
    const float tin_raw = input.ambient_temperature;
    if (!s_ema_initialized) {
      s_tin_ema = tin_raw;
      s_ema_initialized = true;
    } else {
      s_tin_ema = kEMAAlpha * tin_raw + (1.0f - kEMAAlpha) * s_tin_ema;
    }
    const float Tin  = s_tin_ema;
    const float Tset = input.target_temperature;

    // read outdoor temperature if available
    float Tout = 0.0f;
    bool  has_out = false;
    if (outdoor_ambient_temperature_sensor && outdoor_ambient_temperature_sensor->has_state()) {
      Tout = outdoor_ambient_temperature_sensor->state;
      has_out = true;
    }

    // outdoor-aware effective target (don’t try to cool below Tout + margin)
    const float target_floor = has_out ? std::max(Tset, Tout + kOutsideMarginC) : Tset;
    float e = Tin - target_floor;

    // ADDED: debug log of temperatures
    ESP_LOGD("governor", "Temp debug: Tin=%.2f, Tout=%.2f, Tset=%.2f, target_floor=%.2f, error=%.2f", Tin, Tout, Tset, target_floor, e);

    // deadband
    int level = 0;
    if (e > kDeadbandC) {
      float drive = std::clamp(e / kSpanC, 0.0f, 1.0f);
      const float gamma = (input.fan_mode == CLIMATE_FAN_QUIET) ? kGammaQuiet : kGammaAuto;
      float level_f = 10.0f * std::pow(drive, gamma);
      level = static_cast<int>(std::lround(level_f));

      // if outside isn’t cooler, cap to low ventilation
      if (has_out && Tin <= Tout + kOutsideMarginC) {
        level = std::min(level, kMaxVentWhenHotOutside);
      }
    }    switch (input.fan_mode) {
      case CLIMATE_FAN_OFF:
        level = 0;
        break;
      case CLIMATE_FAN_LOW:
        level = 1;
        break;
      case CLIMATE_FAN_QUIET:
        level = std::clamp(level, 0, kMaxQuiet);
        break;
      case CLIMATE_FAN_AUTO:
      default:
        level = std::clamp(level, 0, kMaxAuto);
        break;
    }

    // discrete hysteresis + rate limiting over time
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
    output.lid_open = true;
  } else {
    output.fan_speed = 0;
    output.lid_open = false;
  }

  switch (input.lid_mode) {
    case LidMode::KEEP_OPEN:
      output.lid_open = true;
      break;
    case LidMode::KEEP_CLOSED:
      output.lid_open = false;
      break;
    // NOTE: LidMode::AUTO keeps existing behavior (open when cooling, closed when off)
  }
  return output;
}

} // namespace governor
} // namespace minuet
