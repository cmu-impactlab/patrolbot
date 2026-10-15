// Offline tests for the SBC-owned dock observer and undock controller.
//
// Every case is a scripted sequence of hardware snapshots driven through a small
// simulated base. Nothing here links ARIA, opens a socket, or touches a robot.
//
// Failures report file:line and the case name rather than aborting on the first
// bad assertion, so one run reports every regression.

#include "../undock.hpp"

#include <cmath>
#include <cstdio>
#include <string>

namespace {

const char* g_case = "<none>";
int g_failures = 0;
int g_checks = 0;

#define CHECK(expr)                                                          \
  do {                                                                       \
    ++g_checks;                                                              \
    if (!(expr)) {                                                           \
      std::fprintf(stderr, "FAIL [%s] %s:%d: %s\n", g_case, __FILE__,        \
                   __LINE__, #expr);                                         \
      ++g_failures;                                                          \
    }                                                                        \
  } while (0)

#define CHECK_EQ_INT(actual, expected)                                       \
  do {                                                                       \
    ++g_checks;                                                              \
    const long long a_ = static_cast<long long>(actual);                     \
    const long long e_ = static_cast<long long>(expected);                   \
    if (a_ != e_) {                                                          \
      std::fprintf(stderr, "FAIL [%s] %s:%d: %s == %s (%lld vs %lld)\n",     \
                   g_case, __FILE__, __LINE__, #actual, #expected, a_, e_);  \
      ++g_failures;                                                          \
    }                                                                        \
  } while (0)

#define RUN(name)                                                            \
  g_case = #name;                                                            \
  name();

using namespace patrolbot;

const long kTickMs = 50;

UndockHardwareProfile commissionedProfile() {
  UndockHardwareProfile profile;
  profile.bumper_mapping_valid = true;
  profile.front_bumper_mask = 0x7e00;
  profile.rear_bumper_mask = 0x007e;
  profile.rear_sonar_mapping_valid = true;
  profile.rear_sonar_indices.clear();
  // The three units that actually report on this robot, measured 2026-07-29.
  profile.rear_sonar_indices.push_back(11);
  profile.rear_sonar_indices.push_back(12);
  profile.rear_sonar_indices.push_back(13);
  profile.estop_semantics_valid = true;
  return profile;
}

// ---------------------------------------------------------------------------
// Simulated base
// ---------------------------------------------------------------------------

class FakeBase {
 public:
  FakeBase()
      : profile(commissionedProfile()),
        charge_release_effective(true),
        charge_release_latency_ms(100),
        charge_reassert_after_ms(-1),
        charge_reasserts_when_clear(false),
        contact_break_m(0.20),
        dock_holds_front_bumper(true),
        wheels_locked(false),
        motor_enable_latency_ms(150),
        motor_enable_refused(false),
        motors_drop_at_progress(-1.0),
        rear_range_m(kSonarNoEchoRangeM),
        rear_configured(true),
        rear_fresh(true),
        rear_near_field_unreliable(false),
        rear_bumper(false),
        battery_voltage_override_v(NAN),
        estop(false),
        fault_flags(0),
        stalled(false),
        charge_release_count(0),
        motor_enable_count(0),
        motor_disable_count(0),
        commanded_(0.0),
        charge_zero_at_(-1),
        charge_reassert_at_(-1),
        motor_enable_at_(-1),
        applied_seq_(0),
        seq_applied_twice_(false),
        velocity_with_enable_(false) {
    inputs.now_ms = 0;
    inputs.charge_state = 3;
    inputs.motors_enabled = false;
    inputs.odometer_m = 0.0;
    refreshDerived();
  }

  UndockHardwareProfile profile;
  DockObserver observer;
  UndockController controller;
  UndockInputs inputs;

  // Scriptable hardware behaviour.
  bool charge_release_effective;
  long charge_release_latency_ms;
  long charge_reassert_after_ms;      // -1 disables re-assertion
  bool charge_reasserts_when_clear;   // re-assert even past contact_break_m
  double contact_break_m;             // reverse progress at which contacts part
  bool dock_holds_front_bumper;
  bool wheels_locked;
  long motor_enable_latency_ms;
  bool motor_enable_refused;
  double motors_drop_at_progress;     // negative disables
  double rear_range_m;
  bool rear_configured;
  bool rear_fresh;
  bool rear_near_field_unreliable;
  bool rear_bumper;
  double battery_voltage_override_v;
  bool estop;
  unsigned int fault_flags;
  bool stalled;

  int charge_release_count;
  int motor_enable_count;
  int motor_disable_count;

  UndockOutputs last;

  double progress() const { return -inputs.enc_x_mm / 1000.0; }
  bool contactsMated() const { return progress() < contact_break_m; }
  bool sequenceAppliedTwice() const { return seq_applied_twice_; }
  bool velocityWithEnable() const { return velocity_with_enable_; }

  UndockOutputs tick() {
    inputs.now_ms += kTickMs;

    // Integrate the previously commanded velocity. Motion requires live motors.
    if (inputs.motors_enabled && !wheels_locked && commanded_ != 0.0) {
      const double dt = kTickMs / 1000.0;
      const double step_m = commanded_ * dt;
      const double yaw = inputs.enc_yaw_deg * M_PI / 180.0;
      inputs.enc_x_mm += step_m * 1000.0 * std::cos(yaw);
      inputs.enc_y_mm += step_m * 1000.0 * std::sin(yaw);
      inputs.odometer_m += std::fabs(step_m);
      inputs.linear = commanded_;
    } else {
      inputs.linear = 0.0;
    }
    inputs.angular = 0.0;

    if (charge_zero_at_ >= 0 && inputs.now_ms >= charge_zero_at_) {
      inputs.charge_state = 0;
      charge_zero_at_ = -1;
      if (charge_reassert_after_ms >= 0) {
        charge_reassert_at_ = inputs.now_ms + charge_reassert_after_ms;
      }
    }
    if (charge_reassert_at_ >= 0 && inputs.now_ms >= charge_reassert_at_) {
      if (contactsMated() || charge_reasserts_when_clear) {
        inputs.charge_state = 3;
      }
      charge_reassert_at_ = -1;
    }

    if (motor_enable_at_ >= 0 && inputs.now_ms >= motor_enable_at_) {
      inputs.motors_enabled = true;
      motor_enable_at_ = -1;
    }
    if (motors_drop_at_progress >= 0.0 && progress() >= motors_drop_at_progress) {
      inputs.motors_enabled = false;
    }

    refreshDerived();
    observer.update(inputs, profile);
    last = controller.step(inputs);
    applyEffects(last);
    commanded_ = last.commanded_linear;
    return last;
  }

  void settle(int ticks) {
    for (int i = 0; i < ticks; ++i) tick();
  }

  // Runs until the operation reports a terminal result or the budget expires.
  UndockOutputs runToCompletion(int max_ticks) {
    for (int i = 0; i < max_ticks; ++i) {
      const UndockOutputs out = tick();
      if (out.finished) return out;
    }
    UndockOutputs timed_out;
    timed_out.message = "harness_budget_exhausted";
    return timed_out;
  }

  bool start(double clearance, double speed, double timeout, bool validation,
             int* code, std::string* message) {
    UndockConfig config;
    config.clearance_m = clearance;
    config.speed_mps = speed;
    config.timeout_s = timeout;
    config.validation_mode = validation;
    return controller.request("11111111-2222-3333-4444-555555555555", config,
                              inputs, profile, &observer, code, message);
  }

  bool startDefault() {
    int code = 0;
    std::string message;
    return start(0.53, 0.10, 30.0, false, &code, &message);
  }

 private:
  void refreshDerived() {
    // The real charger holds about 29.7 V. Raw charge state can re-latch after
    // contact breaks, but the off-dock pack falls to about 25.8 V.
    inputs.battery_voltage_v = std::isfinite(battery_voltage_override_v)
        ? battery_voltage_override_v
        : ((contactsMated() && inputs.charge_state > 0) ? 29.7 : 25.8);
    inputs.estop_pressed = estop;
    inputs.fault_flags = fault_flags;
    unsigned int stall = 0;
    if (dock_holds_front_bumper && contactsMated()) stall |= 0x1800;
    if (rear_bumper) stall |= 0x0002;
    if (stalled) stall |= 0x0101;
    inputs.stall_value = stall;

    inputs.rear.configured = rear_configured;
    inputs.rear.all_required_sensors_fresh = rear_fresh;
    inputs.rear.fresh_sensor_count = rear_fresh ? 4u : 0u;
    inputs.rear.minimum_obstacle_range_m = rear_range_m;
    inputs.rear.near_field_unreliable = rear_near_field_unreliable;
  }

  void applyEffects(const UndockOutputs& out) {
    if (out.effect_count == 0) return;
    if (out.effect_seq == applied_seq_) {
      seq_applied_twice_ = true;
      return;
    }
    applied_seq_ = out.effect_seq;
    for (unsigned int i = 0; i < out.effect_count; ++i) {
      switch (out.effects[i]) {
        case EFFECT_SEND_CHARGE_RELEASE:
          ++charge_release_count;
          if (charge_release_effective) {
            charge_zero_at_ = inputs.now_ms + charge_release_latency_ms;
          }
          break;
        case EFFECT_ENABLE_MOTORS:
          ++motor_enable_count;
          if (out.commanded_linear != 0.0) velocity_with_enable_ = true;
          if (!motor_enable_refused) {
            motor_enable_at_ = inputs.now_ms + motor_enable_latency_ms;
          }
          break;
        case EFFECT_DISABLE_MOTORS:
          ++motor_disable_count;
          inputs.motors_enabled = false;
          motor_enable_at_ = -1;
          break;
        default:
          break;
      }
    }
  }

  double commanded_;
  long long charge_zero_at_;
  long long charge_reassert_at_;
  long long motor_enable_at_;
  unsigned long applied_seq_;
  bool seq_applied_twice_;
  bool velocity_with_enable_;
};

// Brings the observer to DOCKED_CONFIRMED without starting an operation.
void settleDocked(FakeBase* base) {
  base->settle(
      static_cast<int>(base->profile.redock_dwell_ms / kTickMs) + 5);
}

// ---------------------------------------------------------------------------
// 1. Normal undock
// ---------------------------------------------------------------------------

void test_normal_undock_succeeds_with_motors_left_enabled() {
  FakeBase base;
  settleDocked(&base);
  CHECK_EQ_INT(base.observer.state(), DOCK_DOCKED_CONFIRMED);
  CHECK(base.startDefault());
  CHECK_EQ_INT(base.observer.state(), DOCK_DEPARTING);

  const UndockOutputs out = base.runToCompletion(1000);
  CHECK(out.finished);
  CHECK(out.success);
  CHECK_EQ_INT(out.code, UNDOCK_OK);
  CHECK_EQ_INT(out.phase, UNDOCK_PHASE_HANDOFF_READY);
  CHECK(out.reverse_progress_m >= 0.53);
  CHECK(out.commanded_linear == 0.0);

  // The defining contract of a successful handoff.
  CHECK(base.inputs.motors_enabled);
  CHECK_EQ_INT(base.observer.state(), DOCK_CLEAR_CONFIRMED);
  CHECK(!base.controller.active());
  CHECK(!base.sequenceAppliedTwice());
  CHECK(!base.velocityWithEnable());
}

// ---------------------------------------------------------------------------
// 2 & 3. Charge re-asserts while reverse motion stays healthy
// ---------------------------------------------------------------------------

void test_charge_reassert_with_healthy_motion_does_not_stop() {
  FakeBase base;
  base.charge_reassert_after_ms = 200;  // re-asserts while contacts are mated
  settleDocked(&base);
  CHECK(base.startDefault());

  bool ever_stopped_while_backing = false;
  bool saw_backing = false;
  bool saw_recovery_after_backing = false;
  for (int i = 0; i < 1000; ++i) {
    const UndockOutputs out = base.tick();
    if (out.phase == UNDOCK_PHASE_BACKING_OUT) {
      saw_backing = true;
      if (out.commanded_linear == 0.0) ever_stopped_while_backing = true;
    }
    if (saw_backing && out.phase == UNDOCK_PHASE_RELEASING) {
      saw_recovery_after_backing = true;
    }
    if (out.finished) {
      CHECK(out.success);
      break;
    }
  }
  CHECK(saw_backing);
  // The whole point of the correction: a healthy reverse keeps going.
  CHECK(!ever_stopped_while_backing);
  CHECK(!saw_recovery_after_backing);
  CHECK_EQ_INT(base.controller.releaseAttempts(), 0);
}

void test_repeated_charge_reassert_respects_resend_rate_limit() {
  FakeBase base;
  base.charge_reassert_after_ms = 100;
  settleDocked(&base);
  CHECK(base.startDefault());

  int releases_during_backing = 0;
  int backing_ticks = 0;
  for (int i = 0; i < 1000; ++i) {
    const int before = base.charge_release_count;
    const UndockOutputs out = base.tick();
    if (out.phase == UNDOCK_PHASE_BACKING_OUT) {
      ++backing_ticks;
      releases_during_backing += base.charge_release_count - before;
    }
    if (out.finished) {
      CHECK(out.success);
      break;
    }
  }
  CHECK(backing_ticks > 0);
  // Rate limited to one per charge_resend_min_ms, never once per 50 ms step.
  const long elapsed_ms = backing_ticks * kTickMs;
  const int ceiling =
      static_cast<int>(elapsed_ms / base.profile.charge_resend_min_ms) + 2;
  CHECK(releases_during_backing <= ceiling);
}

// ---------------------------------------------------------------------------
// 4 & 5. Recovery is entered only when motion is actually interrupted
// ---------------------------------------------------------------------------

void test_motors_dropping_mid_reverse_enters_recovery_and_resumes() {
  FakeBase base;
  base.motors_drop_at_progress = 0.10;
  settleDocked(&base);
  CHECK(base.startDefault());

  bool saw_recovery = false;
  double progress_at_recovery = -1.0;
  for (int i = 0; i < 2000; ++i) {
    const UndockOutputs out = base.tick();
    if (!saw_recovery && out.phase == UNDOCK_PHASE_RELEASING &&
        out.reverse_progress_m > 0.05) {
      saw_recovery = true;
      progress_at_recovery = out.reverse_progress_m;
      // The distance budget carries over rather than restarting.
      base.motors_drop_at_progress = -1.0;
    }
    if (out.finished) {
      CHECK(out.success);
      CHECK(out.reverse_progress_m >= 0.53);
      break;
    }
  }
  CHECK(saw_recovery);
  CHECK(progress_at_recovery > 0.05);
  CHECK(base.controller.releaseAttempts() >= 1);
}

void test_repeated_release_failure_stops_with_motors_disabled() {
  FakeBase base;
  base.motors_drop_at_progress = 0.02;  // never allowed to recover
  settleDocked(&base);
  CHECK(base.startDefault());

  const UndockOutputs out = base.runToCompletion(4000);
  CHECK(out.finished);
  CHECK(!out.success);
  CHECK_EQ_INT(out.phase, UNDOCK_PHASE_STOPPING);
  CHECK(out.commanded_linear == 0.0);
  CHECK(!base.inputs.motors_enabled);
  CHECK_EQ_INT(base.observer.state(), DOCK_FAULT_STOPPED);
  CHECK(base.controller.releaseAttempts() >= base.profile.max_release_attempts);
}

// ---------------------------------------------------------------------------
// 6 & 7. Charge release observation
// ---------------------------------------------------------------------------

void test_ineffective_charge_release_fails_after_bounded_retries() {
  FakeBase base;
  base.charge_release_effective = false;
  settleDocked(&base);
  CHECK(base.startDefault());

  const UndockOutputs out = base.runToCompletion(4000);
  CHECK(out.finished);
  CHECK(!out.success);
  CHECK_EQ_INT(out.code, UNDOCK_RELEASE_FAILED);
  CHECK(!base.inputs.motors_enabled);
  CHECK(base.charge_release_count >= base.profile.max_release_attempts);
  CHECK(base.progress() == 0.0);
}

void test_single_brief_zero_sample_is_caught() {
  FakeBase base;
  // Charge returns to 3 one tick after reaching 0: a multi-sample stability
  // requirement would never see the release.
  base.charge_release_latency_ms = 100;
  base.charge_reassert_after_ms = 50;
  base.contact_break_m = 0.20;
  settleDocked(&base);
  CHECK(base.startDefault());

  const UndockOutputs out = base.runToCompletion(1000);
  CHECK(out.finished);
  CHECK(out.success);
  CHECK(base.inputs.motors_enabled);
}

// ---------------------------------------------------------------------------
// 8 & 9. Motion quality
// ---------------------------------------------------------------------------

// Modelled directly on commission_charge.csv, 2026-07-29: CHARGE 0 -> charge
// reaches 0 after 3552 ms -> re-latches 3072 ms later while still docked.
void test_measured_release_latency_completes_with_a_matching_window() {
  FakeBase base;
  base.profile.release_observe_ms = 5000;
  base.charge_release_latency_ms = 3552;
  base.charge_reassert_after_ms = 3072;
  base.contact_break_m = 0.20;
  settleDocked(&base);
  CHECK(base.startDefault());

  const UndockOutputs out = base.runToCompletion(2000);
  CHECK(out.finished);
  CHECK(out.success);
  CHECK(base.inputs.motors_enabled);
  // The long release must not be mistaken for a failed one.
  CHECK_EQ_INT(base.controller.releaseAttempts(), 0);
}

void test_release_window_shorter_than_the_hardware_latency_fails() {
  // Guards the constant itself. release_observe_ms = 300 was the original
  // default and is twelve times shorter than the measured 3552 ms latency: it
  // burns every retry before the charger ever lets go, and the failure looks
  // like broken hardware rather than a bad number.
  FakeBase base;
  base.profile.release_observe_ms = 300;
  base.charge_release_latency_ms = 3552;
  base.charge_reassert_after_ms = 3072;
  settleDocked(&base);
  CHECK(base.startDefault());

  const UndockOutputs out = base.runToCompletion(2000);
  CHECK(out.finished);
  CHECK(!out.success);
  CHECK_EQ_INT(out.code, UNDOCK_RELEASE_FAILED);
  CHECK(!base.inputs.motors_enabled);
  // It never moved: the failure happened before any reverse was commanded.
  CHECK(base.progress() == 0.0);
}

void test_no_encoder_movement_recovers_then_fails() {
  FakeBase base;
  base.wheels_locked = true;
  settleDocked(&base);
  CHECK(base.startDefault());

  const UndockOutputs out = base.runToCompletion(6000);
  CHECK(out.finished);
  CHECK(!out.success);
  CHECK_EQ_INT(out.code, UNDOCK_MOTION_FAILED);
  CHECK(!base.inputs.motors_enabled);
  CHECK(base.controller.releaseAttempts() >= 1);
}

void test_lateral_drift_beyond_limit_stops() {
  FakeBase base;
  settleDocked(&base);
  CHECK(base.startDefault());
  for (int i = 0; i < 2000; ++i) {
    const UndockOutputs out = base.tick();
    if (out.phase == UNDOCK_PHASE_BACKING_OUT && out.reverse_progress_m > 0.30) {
      base.inputs.enc_y_mm += 200.0;  // 0.2 m sideways, well past the limit
    }
    if (out.finished) {
      CHECK(!out.success);
      CHECK_EQ_INT(out.code, UNDOCK_MOTION_FAILED);
      CHECK(!base.inputs.motors_enabled);
      return;
    }
  }
  CHECK(false);
}

void test_forward_motion_is_never_counted_as_progress() {
  // A Euclidean displacement measure would read a forward lurch off the plates
  // as progress. Signed projection onto the reverse of the departure heading is
  // what makes this a fault.
  FakeBase base;
  settleDocked(&base);
  CHECK(base.startDefault());
  bool injected = false;
  for (int i = 0; i < 2000; ++i) {
    const UndockOutputs out = base.tick();
    if (!injected && out.phase == UNDOCK_PHASE_BACKING_OUT &&
        out.reverse_progress_m > 0.05) {
      base.inputs.enc_x_mm = 100.0;  // 0.1 m FORWARD of where departure began
      injected = true;
    }
    if (injected && out.reverse_progress_m < 0.0) {
      CHECK(out.reverse_progress_m < -0.05);
    }
    if (out.finished) {
      CHECK(!out.success);
      CHECK_EQ_INT(out.code, UNDOCK_MOTION_FAILED);
      CHECK(!base.inputs.motors_enabled);
      return;
    }
  }
  CHECK(false);
}

void test_progress_is_projected_not_euclidean() {
  // A lateral offset inside the drift limit must not inflate the reported
  // distance: a Euclidean measure would, a projection does not.
  FakeBase base;
  settleDocked(&base);
  int code = 0;
  std::string message;
  CHECK(base.start(0.40, 0.10, 30.0, false, &code, &message));

  bool offset_applied = false;
  for (int i = 0; i < 2000; ++i) {
    const UndockOutputs out = base.tick();
    // Applied after the 0.25 m pose guard, so the offset is measured rather
    // than absorbed into the free-running baseline.
    if (!offset_applied && out.phase == UNDOCK_PHASE_BACKING_OUT &&
        out.reverse_progress_m > 0.30) {
      base.inputs.enc_y_mm += 50.0;  // 0.05 m, inside the 0.06 m limit
      offset_applied = true;
    }
    if (out.finished) {
      CHECK(offset_applied);
      CHECK(out.success);
      CHECK(out.lateral_error_m > 0.04);
      const double along_x = -base.inputs.enc_x_mm / 1000.0;
      CHECK(std::fabs(out.reverse_progress_m - along_x) < 1e-6);
      return;
    }
  }
  CHECK(false);
}

// The dock physically twists this robot about 12 degrees while the charger is
// still engaged (measured 2026-07-29, confirmed by AMCL). Policing that against
// the docked pose aborted a departure that was behaving exactly as the fixture
// forces it to.
void test_dock_imposed_yaw_during_capture_does_not_abort() {
  FakeBase base;
  base.contact_break_m = 0.20;
  base.charge_reassert_after_ms = 200;
  settleDocked(&base);
  CHECK(base.startDefault());

  bool twisted = false;
  for (int i = 0; i < 2000; ++i) {
    const UndockOutputs out = base.tick();
    // Twist it past the 0.20 rad limit while the dock still holds it.
    if (!twisted && out.phase == UNDOCK_PHASE_BACKING_OUT &&
        out.reverse_progress_m > 0.03) {
      base.inputs.enc_yaw_deg = 15.0;  // 0.26 rad, more than double the cap
      twisted = true;
    }
    if (twisted && out.phase == UNDOCK_PHASE_BACKING_OUT &&
        out.reverse_progress_m < 0.15) {
      // While captured, the reported deviation is the fixture's, not the
      // controller's, so it must read as zero rather than as a fault.
      CHECK(out.heading_error_rad < 0.01);
    }
    if (out.finished) {
      CHECK(twisted);
      CHECK(out.success);
      CHECK(base.inputs.motors_enabled);
      return;
    }
  }
  CHECK(false);
}

void test_early_charge_zero_and_front_release_do_not_arm_pose_guard() {
  // Reproduces the physical failure: both logical release signals changed
  // before the fixture had finished rotating the robot.
  FakeBase base;
  base.contact_break_m = 0.05;
  settleDocked(&base);
  CHECK(base.startDefault());

  bool twisted = false;
  for (int i = 0; i < 2000; ++i) {
    const UndockOutputs out = base.tick();
    if (!twisted && out.phase == UNDOCK_PHASE_BACKING_OUT &&
        out.reverse_progress_m > 0.10) {
      CHECK_EQ_INT(base.inputs.charge_state, 0);
      CHECK(!frontContactPressed(base.inputs, base.profile));
      base.inputs.enc_yaw_deg = 15.0;
      twisted = true;
    }
    if (twisted && out.phase == UNDOCK_PHASE_BACKING_OUT &&
        out.reverse_progress_m < base.profile.pose_guard_start_m) {
      CHECK(out.heading_error_rad < 0.01);
    }
    if (out.finished) {
      CHECK(twisted);
      CHECK(out.success);
      CHECK(base.inputs.motors_enabled);
      return;
    }
  }
  CHECK(false);
}

void test_stale_charge_does_not_change_pose_guard_start() {
  // The guard boundary must be identical when the controller sees the base's
  // known stale charge=3 report after release.
  FakeBase base;
  base.contact_break_m = 0.05;
  base.charge_reassert_after_ms = 200;
  base.charge_reasserts_when_clear = true;
  settleDocked(&base);
  CHECK(base.startDefault());

  bool twisted = false;
  bool saw_stale_charge = false;
  for (int i = 0; i < 2000; ++i) {
    const UndockOutputs out = base.tick();
    if (!twisted && out.phase == UNDOCK_PHASE_BACKING_OUT &&
        out.reverse_progress_m > 0.10) {
      base.inputs.enc_yaw_deg = 15.0;
      twisted = true;
    }
    if (twisted && base.inputs.charge_state == 3) {
      saw_stale_charge = true;
    }
    if (twisted && out.phase == UNDOCK_PHASE_BACKING_OUT &&
        out.reverse_progress_m < base.profile.pose_guard_start_m) {
      CHECK(out.heading_error_rad < 0.01);
    }
    if (out.finished) {
      CHECK(twisted);
      CHECK(saw_stale_charge);
      CHECK(out.success);
      return;
    }
  }
  CHECK(false);
}

void test_moderate_yaw_after_the_robot_is_free_is_allowed() {
  FakeBase base;
  settleDocked(&base);
  CHECK(base.startDefault());

  bool twisted = false;
  for (int i = 0; i < 2000; ++i) {
    const UndockOutputs out = base.tick();
    if (!twisted && out.phase == UNDOCK_PHASE_BACKING_OUT &&
        out.reverse_progress_m > 0.30) {
      base.inputs.enc_yaw_deg += 10.0;  // 0.17 rad, below the 0.20 rad limit
      twisted = true;
    }
    if (out.finished) {
      CHECK(twisted);
      CHECK(out.success);
      CHECK(out.heading_error_rad > 0.15);
      CHECK(out.heading_error_rad < base.profile.max_departure_yaw_rad);
      return;
    }
  }
  CHECK(false);
}

void test_yaw_after_the_robot_is_free_still_aborts() {
  // The guard must still do its job once the robot is actually driving itself.
  FakeBase base;
  base.contact_break_m = 0.10;
  base.charge_reassert_after_ms = 200;
  settleDocked(&base);
  CHECK(base.startDefault());

  bool twisted = false;
  for (int i = 0; i < 2000; ++i) {
    const UndockOutputs out = base.tick();
    if (!twisted && out.phase == UNDOCK_PHASE_BACKING_OUT &&
        out.reverse_progress_m > 0.30) {
      base.inputs.enc_yaw_deg += 15.0;
      twisted = true;
    }
    if (out.finished) {
      CHECK(twisted);
      CHECK(!out.success);
      CHECK_EQ_INT(out.code, UNDOCK_MOTION_FAILED);
      CHECK(!base.inputs.motors_enabled);
      return;
    }
  }
  CHECK(false);
}

// ---------------------------------------------------------------------------
// 10-13. Rear sensing
// ---------------------------------------------------------------------------

void test_rear_obstacle_during_reverse_stops_immediately() {
  FakeBase base;
  settleDocked(&base);
  CHECK(base.startDefault());
  for (int i = 0; i < 2000; ++i) {
    const UndockOutputs out = base.tick();
    if (out.phase == UNDOCK_PHASE_BACKING_OUT && out.reverse_progress_m > 0.05) {
      base.rear_range_m = 0.10;
    }
    if (out.finished) {
      CHECK(!out.success);
      CHECK_EQ_INT(out.code, UNDOCK_SENSOR_BLOCKED);
      CHECK(!base.inputs.motors_enabled);
      CHECK(out.commanded_linear == 0.0);
      return;
    }
  }
  CHECK(false);
}

void test_insufficient_rear_room_is_refused_before_any_motion() {
  FakeBase base;
  // Enough to avoid an immediate collision, not enough to finish 0.53 m.
  base.rear_range_m = 0.40;
  settleDocked(&base);

  int code = 0;
  std::string message;
  CHECK(!base.start(0.53, 0.10, 30.0, false, &code, &message));
  CHECK_EQ_INT(code, UNDOCK_PRECONDITION_FAILED);
  CHECK(message.find("rear") != std::string::npos);
  CHECK(!base.controller.active());
  CHECK_EQ_INT(base.observer.state(), DOCK_DOCKED_CONFIRMED);
  CHECK_EQ_INT(base.motor_enable_count, 0);
}

void test_stale_rear_sensor_is_never_clear() {
  FakeBase base;
  base.rear_fresh = false;
  settleDocked(&base);
  int code = 0;
  std::string message;
  CHECK(!base.start(0.53, 0.10, 30.0, false, &code, &message));
  CHECK_EQ_INT(code, UNDOCK_PRECONDITION_FAILED);

  FakeBase running;
  settleDocked(&running);
  CHECK(running.startDefault());
  for (int i = 0; i < 2000; ++i) {
    const UndockOutputs out = running.tick();
    if (out.phase == UNDOCK_PHASE_BACKING_OUT && out.reverse_progress_m > 0.05) {
      running.rear_fresh = false;
    }
    if (out.finished) {
      CHECK(!out.success);
      CHECK_EQ_INT(out.code, UNDOCK_SENSOR_BLOCKED);
      return;
    }
  }
  CHECK(false);
}

void test_rear_bumper_stops_immediately() {
  FakeBase base;
  settleDocked(&base);
  CHECK(base.startDefault());
  for (int i = 0; i < 2000; ++i) {
    const UndockOutputs out = base.tick();
    if (out.phase == UNDOCK_PHASE_BACKING_OUT && out.reverse_progress_m > 0.05) {
      base.rear_bumper = true;
    }
    if (out.finished) {
      CHECK(!out.success);
      CHECK_EQ_INT(out.code, UNDOCK_SAFETY_FAULT);
      CHECK(!base.inputs.motors_enabled);
      return;
    }
  }
  CHECK(false);
}

// ---------------------------------------------------------------------------
// 14. Front contact persists
// ---------------------------------------------------------------------------

void test_persistent_front_contact_extends_then_fails() {
  FakeBase base;
  base.contact_break_m = 10.0;  // the dock never lets go
  base.charge_reassert_after_ms = 200;
  settleDocked(&base);
  CHECK(base.startDefault());

  const UndockOutputs out = base.runToCompletion(6000);
  CHECK(out.finished);
  CHECK(!out.success);
  CHECK(!base.inputs.motors_enabled);
  CHECK_EQ_INT(base.observer.state(), DOCK_FAULT_STOPPED);
  // It kept trying past the requested clearance but never past the hard cap.
  CHECK(out.reverse_progress_m >= 0.53);
  CHECK(out.reverse_progress_m <= base.profile.max_clearance_m + 0.05);
}

// ---------------------------------------------------------------------------
// 15 & 16. Stale charge state, and what it takes to be docked again
// ---------------------------------------------------------------------------

void test_stale_charge_after_clearance_keeps_clear_confirmed() {
  FakeBase base;
  base.charge_reassert_after_ms = 200;
  base.charge_reasserts_when_clear = true;  // stale forever
  settleDocked(&base);
  CHECK(base.startDefault());

  const UndockOutputs out = base.runToCompletion(1000);
  CHECK(out.finished);
  CHECK(out.success);
  CHECK_EQ_INT(base.observer.state(), DOCK_CLEAR_CONFIRMED);

  // Ten seconds of a stale charge state must not revoke the proof.
  base.inputs.charge_state = 3;
  base.settle(200);
  CHECK_EQ_INT(base.observer.state(), DOCK_CLEAR_CONFIRMED);
  CHECK(base.inputs.motors_enabled);
}

void test_stale_overcharge_after_clearance_does_not_redock_without_voltage() {
  FakeBase base;
  base.charge_reassert_after_ms = 200;
  base.charge_reasserts_when_clear = true;
  base.profile.redock_dwell_ms = 200;
  settleDocked(&base);
  CHECK(base.startDefault());
  CHECK(base.runToCompletion(1000).success);
  CHECK_EQ_INT(base.observer.state(), DOCK_CLEAR_CONFIRMED);

  // State 2 is no stronger than stale float state 3 after physical clearance.
  base.inputs.charge_state = 2;
  base.settle(20);
  CHECK_EQ_INT(base.observer.state(), DOCK_CLEAR_CONFIRMED);
}

void test_manual_departure_clears_a_stale_float_state() {
  FakeBase base;
  settleDocked(&base);
  base.dock_holds_front_bumper = false;
  base.battery_voltage_override_v = 25.5;
  base.inputs.charge_state = 3;
  base.settle(20);
  CHECK_EQ_INT(base.observer.state(), DOCK_CLEAR_CONFIRMED);
}

void test_charger_voltage_repairs_a_missed_redock_edge() {
  FakeBase base;
  base.charge_reassert_after_ms = 200;
  base.charge_reasserts_when_clear = true;
  base.profile.redock_dwell_ms = 200;
  settleDocked(&base);
  CHECK(base.startDefault());
  CHECK(base.runToCompletion(1000).success);
  CHECK_EQ_INT(base.observer.state(), DOCK_CLEAR_CONFIRMED);

  // The robot returns to the physical contacts while raw charging never
  // produced a new edge. Charger voltage, not state 2/3, repairs the observer.
  base.inputs.enc_x_mm = 0.0;
  base.inputs.enc_y_mm = 0.0;
  base.dock_holds_front_bumper = false;
  base.inputs.charge_state = 2;
  base.settle(6);
  CHECK_EQ_INT(base.observer.state(), DOCK_DOCKED_CONFIRMED);
}

void test_transient_charger_voltage_does_not_redock() {
  FakeBase base;
  base.charge_reassert_after_ms = 200;
  base.charge_reasserts_when_clear = true;
  base.profile.redock_dwell_ms = 200;
  settleDocked(&base);
  CHECK(base.startDefault());
  CHECK(base.runToCompletion(1000).success);

  base.inputs.charge_state = 2;
  base.battery_voltage_override_v = 29.7;
  base.settle(3);
  CHECK_EQ_INT(base.observer.state(), DOCK_CLEAR_CONFIRMED);
  base.battery_voltage_override_v = 25.8;
  base.settle(10);
  CHECK_EQ_INT(base.observer.state(), DOCK_CLEAR_CONFIRMED);
}

void test_redock_requires_a_rising_edge_not_a_stale_level() {
  FakeBase base;
  base.charge_reassert_after_ms = 200;
  base.charge_reasserts_when_clear = true;
  settleDocked(&base);
  CHECK(base.startDefault());
  CHECK(base.runToCompletion(1000).success);
  CHECK_EQ_INT(base.observer.state(), DOCK_CLEAR_CONFIRMED);

  // A stale level, held indefinitely, is not evidence of re-docking.
  base.settle(100);
  CHECK_EQ_INT(base.observer.state(), DOCK_CLEAR_CONFIRMED);

  // A genuine rising edge after a confirmed zero, while stationary and in
  // contact, is.
  base.inputs.charge_state = 0;
  base.settle(10);
  CHECK_EQ_INT(base.observer.state(), DOCK_CLEAR_CONFIRMED);
  base.inputs.charge_state = 3;
  base.dock_holds_front_bumper = true;
  base.contact_break_m = 1e9;  // contact bits asserted again
  base.settle(10);
  CHECK_EQ_INT(base.observer.state(), DOCK_DOCKED_CONFIRMED);
}

void test_redock_by_stationary_dwell_without_contact_bits() {
  FakeBase base;
  base.dock_holds_front_bumper = false;
  settleDocked(&base);
  base.inputs.charge_state = 0;
  base.settle(10);
  CHECK_EQ_INT(base.observer.state(), DOCK_CLEAR_CONFIRMED);

  base.inputs.charge_state = 3;
  base.settle(10);
  // Not yet: the dwell has not elapsed.
  CHECK_EQ_INT(base.observer.state(), DOCK_CLEAR_CONFIRMED);
  base.settle(50);  // 2.5 s total
  CHECK_EQ_INT(base.observer.state(), DOCK_DOCKED_CONFIRMED);
}

// ---------------------------------------------------------------------------
// 17-19. Safety inputs
// ---------------------------------------------------------------------------

void test_estop_from_every_phase_stops_and_disables() {
  const UndockPhase phases[] = {UNDOCK_PHASE_RELEASING,
                                UNDOCK_PHASE_ENABLING_MOTORS,
                                UNDOCK_PHASE_BACKING_OUT};
  for (unsigned int p = 0; p < sizeof(phases) / sizeof(phases[0]); ++p) {
    FakeBase base;
    settleDocked(&base);
    CHECK(base.startDefault());
    bool tripped = false;
    for (int i = 0; i < 2000; ++i) {
      const UndockOutputs out = base.tick();
      if (!tripped && out.phase == phases[p]) {
        base.estop = true;
        tripped = true;
      }
      if (out.finished) {
        CHECK(!out.success);
        CHECK_EQ_INT(out.code, UNDOCK_SAFETY_FAULT);
        CHECK(!base.inputs.motors_enabled);
        CHECK(out.commanded_linear == 0.0);
        break;
      }
    }
    CHECK(tripped);
  }
}

void test_critical_fault_aborts_but_low_battery_does_not() {
  FakeBase base;
  settleDocked(&base);
  CHECK(base.startDefault());
  bool injected = false;
  for (int i = 0; i < 2000; ++i) {
    const UndockOutputs out = base.tick();
    if (!injected && out.phase == UNDOCK_PHASE_BACKING_OUT) {
      base.fault_flags = 0x0008;  // low_battery, bit 3: not critical
      injected = true;
    }
    if (out.finished) {
      CHECK(out.success);
      break;
    }
  }
  CHECK(injected);

  FakeBase critical;
  settleDocked(&critical);
  CHECK(critical.startDefault());
  injected = false;
  for (int i = 0; i < 2000; ++i) {
    const UndockOutputs out = critical.tick();
    if (!injected && out.phase == UNDOCK_PHASE_BACKING_OUT) {
      critical.fault_flags = 0x0100;  // encoder_failure, bit 8
      injected = true;
    }
    if (out.finished) {
      CHECK(!out.success);
      CHECK_EQ_INT(out.code, UNDOCK_SAFETY_FAULT);
      CHECK(!critical.inputs.motors_enabled);
      return;
    }
  }
  CHECK(false);
}

void test_wheel_stall_is_not_treated_as_a_charge_relatch() {
  FakeBase base;
  settleDocked(&base);
  CHECK(base.startDefault());
  bool injected = false;
  for (int i = 0; i < 2000; ++i) {
    const UndockOutputs out = base.tick();
    if (!injected && out.phase == UNDOCK_PHASE_BACKING_OUT &&
        out.reverse_progress_m > 0.05) {
      base.stalled = true;
      injected = true;
    }
    if (out.finished) {
      CHECK(!out.success);
      CHECK_EQ_INT(out.code, UNDOCK_SAFETY_FAULT);
      // Not a release retry: a stall is a hard stop.
      CHECK_EQ_INT(base.controller.releaseAttempts(), 0);
      CHECK(!base.inputs.motors_enabled);
      return;
    }
  }
  CHECK(false);
}

// ---------------------------------------------------------------------------
// 20-22. Session loss and the command lease
// ---------------------------------------------------------------------------

void test_invalidate_terminates_the_operation() {
  FakeBase base;
  settleDocked(&base);
  CHECK(base.startDefault());
  base.settle(20);
  CHECK(base.controller.active());

  base.controller.invalidate("client_session_changed", &base.observer);
  CHECK(!base.controller.active());
  CHECK_EQ_INT(base.observer.state(), DOCK_FAULT_STOPPED);

  const UndockOutputs out = base.tick();
  CHECK(!out.active);
  CHECK(out.commanded_linear == 0.0);
}

void test_lease_age_tracks_the_last_step() {
  FakeBase base;
  settleDocked(&base);
  CHECK(base.startDefault());
  base.settle(4);
  CHECK_EQ_INT(base.controller.leaseAgeMs(base.inputs.now_ms), 0);
  CHECK(base.controller.leaseValid());
  CHECK_EQ_INT(base.controller.leaseAgeMs(base.inputs.now_ms + 300), 300);
}

// ---------------------------------------------------------------------------
// 23-26. Request lifecycle
// ---------------------------------------------------------------------------

void test_cancel_is_terminal_only_after_stop_and_disable() {
  FakeBase base;
  settleDocked(&base);
  CHECK(base.startDefault());
  base.settle(40);
  CHECK(base.controller.active());

  CHECK(base.controller.cancel("11111111-2222-3333-4444-555555555555"));
  // Still active: cancellation is not complete until the base is stopped.
  CHECK(base.controller.active());

  const UndockOutputs out = base.runToCompletion(200);
  CHECK(out.finished);
  CHECK(!out.success);
  CHECK_EQ_INT(out.code, UNDOCK_CANCELED);
  CHECK(out.commanded_linear == 0.0);
  CHECK(!base.inputs.motors_enabled);
  CHECK_EQ_INT(base.observer.state(), DOCK_FAULT_STOPPED);
}

void test_cancel_with_unknown_id_is_refused() {
  FakeBase base;
  settleDocked(&base);
  CHECK(base.startDefault());
  CHECK(!base.controller.cancel("99999999-9999-4999-8999-999999999999"));
  CHECK(base.controller.active());
}

void test_second_request_while_active_is_busy() {
  FakeBase base;
  settleDocked(&base);
  CHECK(base.startDefault());
  base.settle(10);

  UndockConfig config;
  config.clearance_m = 0.30;
  config.speed_mps = 0.05;
  config.timeout_s = 20.0;
  int code = 0;
  std::string message;
  CHECK(!base.controller.request("aaaaaaaa-bbbb-4ccc-8ddd-eeeeeeeeeeee", config,
                                 base.inputs, base.profile, &base.observer,
                                 &code, &message));
  CHECK_EQ_INT(code, UNDOCK_BUSY);
  CHECK(base.controller.active());
}

void test_goals_outside_hard_caps_are_rejected_not_clamped() {
  FakeBase base;
  settleDocked(&base);
  int code = 0;
  std::string message;
  CHECK(!base.start(0.90, 0.10, 30.0, false, &code, &message));
  CHECK_EQ_INT(code, UNDOCK_INVALID_REQUEST);
  CHECK(!base.start(0.53, 0.25, 30.0, false, &code, &message));
  CHECK_EQ_INT(code, UNDOCK_INVALID_REQUEST);
  CHECK(!base.start(0.0, 0.10, 30.0, false, &code, &message));
  CHECK_EQ_INT(code, UNDOCK_INVALID_REQUEST);
  CHECK(!base.start(0.53, 0.10, 900.0, false, &code, &message));
  CHECK_EQ_INT(code, UNDOCK_INVALID_REQUEST);
  CHECK(!base.controller.active());
}

// ---------------------------------------------------------------------------
// 27-29. Effects, boundary and timeout
// ---------------------------------------------------------------------------

void test_enable_motors_never_shares_a_step_with_reverse_velocity() {
  FakeBase base;
  settleDocked(&base);
  CHECK(base.startDefault());
  for (int i = 0; i < 1000; ++i) {
    const UndockOutputs out = base.tick();
    for (unsigned int e = 0; e < out.effect_count; ++e) {
      if (out.effects[e] == EFFECT_ENABLE_MOTORS) {
        CHECK(out.commanded_linear == 0.0);
      }
    }
    if (out.finished) break;
  }
  CHECK(!base.velocityWithEnable());
  // One enable for the single successful release-and-enable transition.
  CHECK_EQ_INT(base.motor_enable_count, 1);
  CHECK(!base.sequenceAppliedTwice());
}

void test_exact_clearance_boundary_completes() {
  FakeBase base;
  base.contact_break_m = 0.05;
  settleDocked(&base);
  int code = 0;
  std::string message;
  CHECK(base.start(0.20, 0.10, 30.0, false, &code, &message));
  const UndockOutputs out = base.runToCompletion(1000);
  CHECK(out.success);
  CHECK(out.reverse_progress_m >= 0.20);
  // Bounded: it stops promptly rather than running to the hard cap.
  CHECK(out.reverse_progress_m < 0.30);
}

void test_overall_timeout_stops_and_disables() {
  FakeBase base;
  base.contact_break_m = 10.0;
  base.charge_reassert_after_ms = 200;
  settleDocked(&base);
  int code = 0;
  std::string message;
  CHECK(base.start(0.60, 0.02, 3.0, false, &code, &message));
  const UndockOutputs out = base.runToCompletion(4000);
  CHECK(out.finished);
  CHECK(!out.success);
  CHECK_EQ_INT(out.code, UNDOCK_TIMEOUT);
  CHECK(!base.inputs.motors_enabled);
  CHECK(out.commanded_linear == 0.0);
}

// ---------------------------------------------------------------------------
// 30. Hardware profile gate
// ---------------------------------------------------------------------------

void test_uncommissioned_profile_refuses_production_but_allows_validation() {
  FakeBase base;
  base.profile.rear_sonar_mapping_valid = false;
  base.profile.estop_semantics_valid = false;
  settleDocked(&base);

  int code = 0;
  std::string message;
  CHECK(!base.start(0.53, 0.10, 30.0, false, &code, &message));
  CHECK_EQ_INT(code, UNDOCK_HARDWARE_PROFILE_UNCOMMISSIONED);

  CHECK(base.start(0.10, 0.05, 20.0, true, &code, &message));
  CHECK(base.controller.active());

  // Validation mode has its own caps.
  FakeBase over;
  over.profile.rear_sonar_mapping_valid = false;
  settleDocked(&over);
  CHECK(!over.start(0.53, 0.05, 20.0, true, &code, &message));
  CHECK_EQ_INT(code, UNDOCK_INVALID_REQUEST);
  CHECK(!over.start(0.10, 0.10, 20.0, true, &code, &message));
  CHECK_EQ_INT(code, UNDOCK_INVALID_REQUEST);
}

// ---------------------------------------------------------------------------
// 31. Redock inhibition
// ---------------------------------------------------------------------------

void test_redock_inhibition_needs_both_time_and_distance() {
  FakeBase base;
  settleDocked(&base);
  CHECK(!base.observer.redockInhibited(base.inputs, base.profile));
  CHECK(base.startDefault());
  CHECK(base.runToCompletion(1000).success);

  // Clear by 0.53 m but only seconds old: still inhibited.
  CHECK(base.observer.redockInhibited(base.inputs, base.profile));
  CHECK(base.observer.redockInhibitRemainingMs(base.inputs, base.profile) > 0);

  base.inputs.now_ms += base.profile.redock_inhibit_ms + 1000;
  CHECK(!base.observer.redockInhibited(base.inputs, base.profile));

  // Time elapsed but the robot has been pushed back to the dock: inhibited again.
  base.inputs.enc_x_mm = 0.0;
  CHECK(base.observer.redockInhibited(base.inputs, base.profile));
}

// ---------------------------------------------------------------------------
// 32-34. Restart and fault recovery
// ---------------------------------------------------------------------------

void test_restart_while_docked_starts_unknown_and_never_drives() {
  FakeBase base;
  CHECK_EQ_INT(base.observer.state(), DOCK_UNKNOWN);
  base.settle(
      static_cast<int>(base.profile.redock_dwell_ms / kTickMs) - 1);
  CHECK_EQ_INT(base.observer.state(), DOCK_UNKNOWN);
  base.settle(6);
  CHECK_EQ_INT(base.observer.state(), DOCK_DOCKED_CONFIRMED);
  CHECK_EQ_INT(base.motor_enable_count, 0);
  CHECK(!base.inputs.motors_enabled);
}

void test_undock_from_unknown_is_rejected() {
  FakeBase base;
  // No settling: the observer has proven nothing yet.
  int code = 0;
  std::string message;
  CHECK(!base.start(0.53, 0.10, 30.0, false, &code, &message));
  CHECK_EQ_INT(code, UNDOCK_PRECONDITION_FAILED);
  CHECK(!base.controller.active());
}

void test_fresh_controller_mid_manoeuvre_does_not_resume() {
  FakeBase base;
  settleDocked(&base);
  CHECK(base.startDefault());
  base.settle(40);
  CHECK(base.controller.active());

  // Model a process restart: brand-new observer and controller against the same
  // hardware state.
  UndockController fresh_controller;
  DockObserver fresh_observer;
  CHECK(!fresh_controller.active());
  CHECK_EQ_INT(fresh_observer.state(), DOCK_UNKNOWN);
  const UndockOutputs out = fresh_controller.step(base.inputs);
  CHECK(!out.active);
  CHECK(out.commanded_linear == 0.0);
  CHECK_EQ_INT(out.effect_count, 0);
}

void test_fault_stopped_is_cleared_only_by_reset() {
  FakeBase base;
  settleDocked(&base);
  CHECK(base.startDefault());
  base.settle(20);
  base.controller.invalidate("client_session_changed", &base.observer);
  CHECK_EQ_INT(base.observer.state(), DOCK_FAULT_STOPPED);

  // Observation alone never clears it, no matter what charging does.
  base.inputs.charge_state = 0;
  base.settle(100);
  CHECK_EQ_INT(base.observer.state(), DOCK_FAULT_STOPPED);

  // Nor does a reset while the underlying condition persists.
  base.estop = true;
  base.settle(2);
  std::string error;
  CHECK(!base.observer.reset(base.inputs, base.profile, &error));
  CHECK_EQ_INT(base.observer.state(), DOCK_FAULT_STOPPED);

  base.estop = false;
  base.settle(2);
  CHECK(base.observer.reset(base.inputs, base.profile, &error));
  CHECK_EQ_INT(base.observer.state(), DOCK_UNKNOWN);
}

// ---------------------------------------------------------------------------
// Bumper mask regression (mapping is established; this guards it)
// ---------------------------------------------------------------------------

void test_commissioned_bumper_masks_decode_each_segment() {
  const UndockHardwareProfile profile = commissionedProfile();
  UndockInputs inputs;

  for (int bit = 9; bit <= 14; ++bit) {
    inputs.stall_value = 1u << bit;
    CHECK(frontContactPressed(inputs, profile));
    CHECK(!rearBumperPressed(inputs, profile));
  }
  for (int bit = 1; bit <= 6; ++bit) {
    inputs.stall_value = 1u << bit;
    CHECK(rearBumperPressed(inputs, profile));
    CHECK(!frontContactPressed(inputs, profile));
  }
  // Motor stall bits belong to neither bumper.
  inputs.stall_value = 0x0101;
  CHECK(!frontContactPressed(inputs, profile));
  CHECK(!rearBumperPressed(inputs, profile));
  CHECK(wheelStalled(inputs));

  inputs.stall_value = 0;
  CHECK(!frontContactPressed(inputs, profile));
  CHECK(!rearBumperPressed(inputs, profile));
  CHECK(!wheelStalled(inputs));

  // An uncommissioned mapping never reports a released bumper.
  UndockHardwareProfile unmapped;
  unmapped.bumper_mapping_valid = false;
  inputs.stall_value = 0;
  CHECK(!frontContactPressed(inputs, unmapped));
  CHECK(!rearBumperPressed(inputs, unmapped));
}

void test_rear_sonar_status_usability() {
  RearSonarStatus rear;
  CHECK(!rear.usable());  // default-constructed is never clear

  rear.configured = true;
  rear.all_required_sensors_fresh = true;
  rear.minimum_obstacle_range_m = kSonarNoEchoRangeM;
  CHECK(rear.usable());

  rear.near_field_unreliable = true;
  CHECK(!rear.usable());
  rear.near_field_unreliable = false;

  rear.all_required_sensors_fresh = false;
  CHECK(!rear.usable());
  rear.all_required_sensors_fresh = true;

  rear.configured = false;
  CHECK(!rear.usable());
}

}  // namespace

int main() {
  RUN(test_normal_undock_succeeds_with_motors_left_enabled);
  RUN(test_charge_reassert_with_healthy_motion_does_not_stop);
  RUN(test_repeated_charge_reassert_respects_resend_rate_limit);
  RUN(test_motors_dropping_mid_reverse_enters_recovery_and_resumes);
  RUN(test_repeated_release_failure_stops_with_motors_disabled);
  RUN(test_ineffective_charge_release_fails_after_bounded_retries);
  RUN(test_single_brief_zero_sample_is_caught);
  RUN(test_measured_release_latency_completes_with_a_matching_window);
  RUN(test_release_window_shorter_than_the_hardware_latency_fails);
  RUN(test_no_encoder_movement_recovers_then_fails);
  RUN(test_lateral_drift_beyond_limit_stops);
  RUN(test_forward_motion_is_never_counted_as_progress);
  RUN(test_progress_is_projected_not_euclidean);
  RUN(test_dock_imposed_yaw_during_capture_does_not_abort);
  RUN(test_early_charge_zero_and_front_release_do_not_arm_pose_guard);
  RUN(test_stale_charge_does_not_change_pose_guard_start);
  RUN(test_moderate_yaw_after_the_robot_is_free_is_allowed);
  RUN(test_yaw_after_the_robot_is_free_still_aborts);
  RUN(test_rear_obstacle_during_reverse_stops_immediately);
  RUN(test_insufficient_rear_room_is_refused_before_any_motion);
  RUN(test_stale_rear_sensor_is_never_clear);
  RUN(test_rear_bumper_stops_immediately);
  RUN(test_persistent_front_contact_extends_then_fails);
  RUN(test_stale_charge_after_clearance_keeps_clear_confirmed);
  RUN(test_stale_overcharge_after_clearance_does_not_redock_without_voltage);
  RUN(test_manual_departure_clears_a_stale_float_state);
  RUN(test_charger_voltage_repairs_a_missed_redock_edge);
  RUN(test_transient_charger_voltage_does_not_redock);
  RUN(test_redock_requires_a_rising_edge_not_a_stale_level);
  RUN(test_redock_by_stationary_dwell_without_contact_bits);
  RUN(test_estop_from_every_phase_stops_and_disables);
  RUN(test_critical_fault_aborts_but_low_battery_does_not);
  RUN(test_wheel_stall_is_not_treated_as_a_charge_relatch);
  RUN(test_invalidate_terminates_the_operation);
  RUN(test_lease_age_tracks_the_last_step);
  RUN(test_cancel_is_terminal_only_after_stop_and_disable);
  RUN(test_cancel_with_unknown_id_is_refused);
  RUN(test_second_request_while_active_is_busy);
  RUN(test_goals_outside_hard_caps_are_rejected_not_clamped);
  RUN(test_enable_motors_never_shares_a_step_with_reverse_velocity);
  RUN(test_exact_clearance_boundary_completes);
  RUN(test_overall_timeout_stops_and_disables);
  RUN(test_uncommissioned_profile_refuses_production_but_allows_validation);
  RUN(test_redock_inhibition_needs_both_time_and_distance);
  RUN(test_restart_while_docked_starts_unknown_and_never_drives);
  RUN(test_undock_from_unknown_is_rejected);
  RUN(test_fresh_controller_mid_manoeuvre_does_not_resume);
  RUN(test_fault_stopped_is_cleared_only_by_reset);
  RUN(test_commissioned_bumper_masks_decode_each_segment);
  RUN(test_rear_sonar_status_usability);

  if (g_failures != 0) {
    std::fprintf(stderr, "\nundock_test: %d of %d checks FAILED\n", g_failures,
                 g_checks);
    return 1;
  }
  std::fprintf(stderr, "undock_test: %d checks passed\n", g_checks);
  return 0;
}
