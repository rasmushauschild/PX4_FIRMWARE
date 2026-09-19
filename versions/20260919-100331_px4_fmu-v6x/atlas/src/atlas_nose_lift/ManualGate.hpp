#pragma once
#include <cmath>
#include <cstdint>
// No position input: this mode must not silently depend on GNSS or a local origin.
struct AtlasManualGate {
 static bool simulation_allowed(bool sitl, int sys_hitl, bool hil_active, bool status_fresh) {
  return sitl || (sys_hitl==1 && hil_active && status_fresh);
 }
 enum State { Idle, Prime, Nose, Flight, Failed };
 State state{Idle};
 uint64_t stable_since{};
 float thrust{};
 bool start(bool disarmed, bool landed, bool fresh, float throttle) {
  if ((state!=Idle && state!=Failed) || !disarmed || !landed || !fresh || !std::isfinite(throttle) || throttle<0 || throttle>.05f) { return false; }
  state=Prime; stable_since=0; thrust=0; return true;
 }
 bool confirm(bool press, bool stable, uint64_t now) {
  if (state!=Nose) { return false; }
  if (!stable) { stable_since=0; return false; }
  if (!stable_since) { stable_since=now; }
  if (press && now-stable_since>=1000000) { state=Flight; thrust=0; return true; }
  return false;
 }
 float slew(float target, float dt) {
  if (!std::isfinite(target) || !std::isfinite(dt) || dt<=0) { return thrust; }
  target=fmaxf(0.f,fminf(1.f,target));
  dt=fminf(dt,.02f);
  // Pilot reductions are immediate; increases take at least five seconds full scale.
  thrust=target<thrust ? target : fminf(target,thrust+.2f*dt);
  return thrust;
 }
};
