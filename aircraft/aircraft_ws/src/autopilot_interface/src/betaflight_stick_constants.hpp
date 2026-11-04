#ifndef AUTOPILOT_INTERFACE__BETAFLIGHT_STICK_CONSTANTS_HPP_
#define AUTOPILOT_INTERFACE__BETAFLIGHT_STICK_CONSTANTS_HPP_

#include <cmath>

// Stick scales of betaflight_interface and betaflight_rc_override, from betaflight-2026.6.cli (linear stick curves) and Betaflight's defaults
constexpr double MAX_RATE_RAD_S = 670.0 * M_PI / 180.0; // Rate at full stick (rc_rate = srate = 67)
constexpr double ANGLE_LIMIT_RAD = 60.0 * M_PI / 180.0; // Tilt at full roll or pitch stick in ANGLE (angle_limit, default 60)
constexpr double FULL_STICK_SPEED_MS = 5.0; // Horizontal velocity at full roll or pitch stick in POS HOLD (ap_max_velocity, default 500 cm/s)
constexpr double FULL_STICK_CLIMB_MS = 5.0; // Climb rate at full throttle in ALTHOLD (alt_hold_climb_rate, default 50)
constexpr double MIN_CHECK_US = 1050.0; // RC pulse width, Betaflight rescales the throttle from [min_check, 2000] to [1000, 2000]
constexpr double AP_HOVER_THROTTLE_US = 1595.0; // ap_hover_throttle, measured on the simulated Iris in ANGLE
constexpr double HOVER_THROTTLE = (AP_HOVER_THROTTLE_US - 1000.0) / 1000.0; // Throttle stick in [0, 1] that hovers in ACRO and ANGLE

#endif // AUTOPILOT_INTERFACE__BETAFLIGHT_STICK_CONSTANTS_HPP_
