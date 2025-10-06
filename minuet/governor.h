// MINUET GOVERNOR MODULE
//
// Determines the fan speed and lid state under automatic thermostat control
// based on the thermostat state, user preferences, and environmental sensors.
#pragma once

#include <cstdint>

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
  // - `OPEN`: keep the lid open for passive ventilation
  // - `CLOSED`: keep the lid closed for ceiling fan mode
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
}

// Determines the next set of control outputs from the provided inputs and sensors.
// Called when the inputs change and periodically.
ControlOutput update(ControlInput input) {
  ControlOutput output{};
  if (input.action == ClimateAction::CLIMATE_ACTION_COOLING) {
    // Determine an appropriate fan speed to enhance the thermal comfort of the occupants based on the
    // difference between the ambient temperature and target temperature and the user's specified fan mode.
    // We don't have a lot of information for this calculation and the target temperature may not be
    // achievable when it's too hot outdoors so a PID controller may not be of much use here as it would
    // likely always saturate.  So the goal is simply to provide enough airflow efficiently.
    //
    // TODO: Choose a less arbitrary transfer function.
    // TODO: Control the fan speed over a continuous range of values instead of in discrete steps?
    const float delta = input.ambient_temperature - input.target_temperature;
    output.fan_speed = delta > 0.f ? int(std::ceil(delta / 2.f)) : 0;
    switch (input.fan_mode) {
      case ClimateFanMode::CLIMATE_FAN_OFF:
        output.fan_speed = 0;
        break;
      case ClimateFanMode::CLIMATE_FAN_LOW:
        output.fan_speed = 1;
        break;
      case ClimateFanMode::CLIMATE_FAN_QUIET:
        output.fan_speed = std::clamp(output.fan_speed, 1, 3);
        break;
      case ClimateFanMode::CLIMATE_FAN_AUTO:
      default:
        output.fan_speed = std::clamp(output.fan_speed, 1, 6);
        break;
    }
    output.lid_open = true;
  } else {
    output.fan_speed = 0;
    output.lid_open = false;
  }

  switch (input.lid_mode) {
    case LidMode::OPEN:
      output.lid_open = true;
      break;
    case LidMode::CLOSED:
      output.lid_open = false;
      break;
  }
  return output;
}

} // namespace governor
} // namespace minuet
