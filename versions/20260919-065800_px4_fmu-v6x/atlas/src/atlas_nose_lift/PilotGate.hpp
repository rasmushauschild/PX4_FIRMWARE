#pragma once
#include <cmath>
// Kept independent of PX4 so loss, low-start and reduction behaviour can be tested without motors.
struct AtlasPilotGate {
 float peak{0.f};
 bool raised{false};
 void reset() { peak=0.f; raised=false; }
 static bool can_start(bool valid, float throttle) {
  return valid && std::isfinite(throttle) && throttle >= 0.f && throttle <= .05f;
 }
 static bool may_liftoff(bool stable, bool second_press, bool valid) { return stable && second_press && valid; }
 bool update(bool valid, float throttle) {
  if (!valid || !std::isfinite(throttle) || throttle < 0.f || throttle > 1.f) { return false; }
  if (throttle > peak) { peak=throttle; }
  if (peak > .10f) { raised=true; }
  // Reducing the stick requests controlled lowering/landing, never an airborne power cut.
  return !(raised && (throttle < .05f || peak-throttle >= .05f));
 }
};
