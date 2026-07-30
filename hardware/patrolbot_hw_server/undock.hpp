#ifndef PATROLBOT_UNDOCK_HPP
#define PATROLBOT_UNDOCK_HPP

// Exclusive, SBC-owned dock-departure controller.
//
// This header and undock.cpp deliberately include no ARIA headers and perform no
// I/O. Everything here is a pure function of an explicit hardware snapshot, so the
// whole state machine is unit-testable off the robot against scripted snapshot
// sequences. patrolbot_server.cpp is the only translation unit that maps ArRobot
// onto these structures and applies the resulting effects.
//
// Two separate machines live here:
//
//   DockObserver     persistent; decides whether the robot may be driven at all.
//   UndockController bounded; owns motion for the duration of one UNDOCK request.
//
// The SBC never commands angular velocity. There is no turn phase and no
// commanded_angular field: the collision-aware turn away from the dock stays in
// Nav2, executed after this controller hands authority back.

#include <string>
#include <vector>

namespace patrolbot {

// ---------------------------------------------------------------------------
// Persistent dock observer state
// ---------------------------------------------------------------------------

enum DockPhysicalState {
  // Nothing proven. The state at process start; driving is refused.
  DOCK_UNKNOWN = 0,
  // Charging and/or front-contact evidence. Driving is refused.
  DOCK_DOCKED_CONFIRMED,
  // An UNDOCK operation owns motion.
  DOCK_DEPARTING,
  // Proven physically clear. The only state in which ordinary DRIVE is allowed.
  DOCK_CLEAR_CONFIRMED,
  // Stopped with motors disabled after a failed or cancelled undock. Cleared
  // only by an explicit DOCK_RESET once the underlying condition has gone.
  DOCK_FAULT_STOPPED
};

inline const char* dockStateName(DockPhysicalState state) {
  switch (state) {
    case DOCK_DOCKED_CONFIRMED: return "DOCKED_CONFIRMED";
    case DOCK_DEPARTING:        return "DEPARTING";
    case DOCK_CLEAR_CONFIRMED:  return "CLEAR_CONFIRMED";
    case DOCK_FAULT_STOPPED:    return "FAULT_STOPPED";
    default:                    return "UNKNOWN";
  }
}

// ---------------------------------------------------------------------------
// Bounded undock operation phases
// ---------------------------------------------------------------------------

enum UndockPhase {
  UNDOCK_PHASE_IDLE = 0,
  UNDOCK_PHASE_PREPARING,
  UNDOCK_PHASE_RELEASING,
  UNDOCK_PHASE_ENABLING_MOTORS,
  UNDOCK_PHASE_BACKING_OUT,
  UNDOCK_PHASE_VERIFYING_CLEARANCE,
  UNDOCK_PHASE_HANDOFF_READY,
  UNDOCK_PHASE_STOPPING
};

inline const char* undockPhaseName(UndockPhase phase) {
  switch (phase) {
    case UNDOCK_PHASE_PREPARING:           return "PREPARING_UNDOCK";
    case UNDOCK_PHASE_RELEASING:           return "RELEASING";
    case UNDOCK_PHASE_ENABLING_MOTORS:     return "ENABLING_MOTORS";
    case UNDOCK_PHASE_BACKING_OUT:         return "BACKING_OUT";
    case UNDOCK_PHASE_VERIFYING_CLEARANCE: return "VERIFYING_CLEARANCE";
    case UNDOCK_PHASE_HANDOFF_READY:       return "HANDOFF_READY";
    case UNDOCK_PHASE_STOPPING:            return "STOPPING";
    default:                               return "IDLE";
  }
}

// ---------------------------------------------------------------------------
// Result codes. HardwareUndock.action mirrors these 1:1.
// ---------------------------------------------------------------------------

enum UndockCode {
  UNDOCK_OK = 0,
  UNDOCK_INVALID_REQUEST = 1,
  UNDOCK_LINK_UNAVAILABLE = 2,   // Pi-side only; never produced by the SBC.
  UNDOCK_PRECONDITION_FAILED = 3,
  UNDOCK_BUSY = 4,
  UNDOCK_TIMEOUT = 5,
  UNDOCK_SESSION_CHANGED = 6,
  UNDOCK_PROTOCOL_ERROR = 7,     // Pi-side only; never produced by the SBC.
  UNDOCK_RELEASE_FAILED = 8,
  UNDOCK_MOTOR_STATE_FAILED = 9,
  UNDOCK_MOTION_FAILED = 10,
  UNDOCK_SENSOR_BLOCKED = 11,
  UNDOCK_SAFETY_FAULT = 12,
  UNDOCK_CANCELED = 13,
  UNDOCK_HARDWARE_PROFILE_UNCOMMISSIONED = 14
};

// ---------------------------------------------------------------------------
// Rear sonar
// ---------------------------------------------------------------------------

// Range reported when every fresh rear sensor sees no echo. Chosen far above any
// physically meaningful clearance so "no obstacle" never blocks a feasibility
// check, while remaining a finite number the wire protocol can carry.
const double kSonarNoEchoRangeM = 99.0;

struct RearSonarStatus {
  RearSonarStatus();

  // The profile supplies validated rear transducer indices. False means the rear
  // arc is uncommissioned; it is never treated as clear.
  bool configured;
  // Every configured index produced a reading inside the freshness window. A
  // missing or stale sensor is never "clear".
  bool all_required_sensors_fresh;
  unsigned int fresh_sensor_count;
  // Minimum range over fresh sensors reporting a real echo, or kSonarNoEchoRangeM
  // when every fresh sensor reports no return.
  double minimum_obstacle_range_m;
  // At least one fresh sensor read below the profile's near-field floor, so its
  // value cannot be trusted. Blocks motion.
  bool near_field_unreliable;

  // Usable only when configured, fully fresh and not reading into the near field.
  bool usable() const;
};

// ---------------------------------------------------------------------------
// Commissioned hardware profile and deployment policy
// ---------------------------------------------------------------------------

struct UndockHardwareProfile {
  // Constructs conservative built-in defaults with every *_valid flag false, so a
  // production undock is refused until a profile file supplies validated values.
  UndockHardwareProfile();

  // --- commissioned hardware facts -----------------------------------------
  // Bumper mapping is ESTABLISHED: physically pressed segment by segment on
  // 2026-07-23 (front 0x7e00 = SIP stall bits 9-14, rear 0x007e = bits 1-6),
  // consistent with ARIA's high-byte-front / low-byte-rear convention. The
  // remaining work on bumpers is a functional regression test, not discovery.
  bool bumper_mapping_valid;
  unsigned int front_bumper_mask;
  unsigned int rear_bumper_mask;

  // PENDING commissioning: which ARIA sonar indices are the rear transducers,
  // their reliable near-field floor, and the in-motion stop range.
  bool rear_sonar_mapping_valid;
  std::vector<int> rear_sonar_indices;
  double rear_near_field_floor_m;
  double rear_stop_range_m;
  // Age beyond which a rear transducer reading is stale. A stale sensor is
  // never "clear"; it blocks motion.
  long rear_sonar_fresh_ms;

  // PENDING commissioning: whether isEStopPressed() reports the physical red
  // button or a charging/motor inhibit. Until validated the undock is refused,
  // because a docked robot has been observed reporting estop=1 with motors=0.
  bool estop_semantics_valid;

  // Explicit masks over the 16-bit FAULTFLAGS word. Deliberately not "any nonzero":
  // a docked robot legitimately reports low_battery.
  unsigned int critical_fault_mask;
  unsigned int precheck_blocking_fault_mask;

  // --- commissioned timing --------------------------------------------------
  // How long to watch for a charge release before retrying. The controller
  // accepts a falling edge or a single valid zero sample; it deliberately does
  // not require a multi-sample dwell, because charge_state is known to reach 0
  // only briefly.
  long release_observe_ms;
  double min_motion_m;
  long motion_check_ms;

  // --- motion limits --------------------------------------------------------
  double max_clearance_m;
  double max_speed_mps;
  double max_timeout_s;
  double max_lateral_drift_m;
  double max_departure_yaw_rad;
  // Reverse progress at which the pose guard engages. Charge state and front
  // bumper release cannot engage it early: both can change while the fixture
  // is still imposing the robot's departure heading.
  double pose_guard_start_m;
  double braking_margin_m;
  double sonar_uncertainty_m;
  long charge_resend_min_ms;
  int max_release_attempts;
  // Applies to the persistent observer only. It is NOT part of the release path.
  int debounce_cycles;
  long lease_max_age_ms;
  // How far past the requested clearance VERIFYING_CLEARANCE may extend when the
  // front contact is still engaged, per attempt.
  double clearance_extension_m;
  // Stationary dwell of corroborated charger voltage required to declare the
  // robot docked again when no contact bit is asserted.
  long redock_dwell_ms;
  // Minimum real battery voltage that corroborates physical charger power.
  // Raw charge_state can re-latch after departure and is never sufficient.
  double redock_min_voltage_v;

  // --- validation-mode caps -------------------------------------------------
  double validation_max_clearance_m;
  double validation_max_speed_mps;

  // --- deployment policy ----------------------------------------------------
  long redock_inhibit_ms;
  double redock_inhibit_m;

  // True only when every physically validated fact is present.
  bool commissioned() const;
};

// ---------------------------------------------------------------------------
// One undock request
// ---------------------------------------------------------------------------

struct UndockConfig {
  UndockConfig();

  double clearance_m;
  double speed_mps;
  double timeout_s;
  // Bounded, operator-supervised mode. The only way to run the real operation
  // against an uncommissioned profile, so the commissioning session itself can
  // exercise it.
  bool validation_mode;
};

// ---------------------------------------------------------------------------
// One hardware snapshot
// ---------------------------------------------------------------------------

struct UndockInputs {
  UndockInputs();

  // Monotonic milliseconds since process start. long long so a server running for
  // years cannot overflow it.
  long long now_ms;

  int charge_state;
  double battery_voltage_v;
  bool motors_enabled;
  bool estop_pressed;
  unsigned int fault_flags;
  unsigned int stall_value;

  // Raw encoder pose. ArPose reports X/Y in MILLIMETRES and heading in DEGREES;
  // both are carried here in their native units and converted exactly once, in
  // undock.cpp, so the conversion cannot be duplicated or lost.
  double enc_x_mm;
  double enc_y_mm;
  double enc_yaw_deg;
  // Monotonic path length in metres. Used only to corroborate that the wheels
  // turned at all; never as the completion measurement.
  double odometer_m;

  double linear;   // m/s, reported
  double angular;  // rad/s, reported

  RearSonarStatus rear;
};

// Bumper and stall decoding. Returns false whenever the mapping is uncommissioned,
// so an unmapped bumper is never mistaken for a released one.
bool frontContactPressed(const UndockInputs& inputs,
                         const UndockHardwareProfile& profile);
bool rearBumperPressed(const UndockInputs& inputs,
                       const UndockHardwareProfile& profile);
// ARIA packs the left wheel stall in the high byte and the right in the low byte;
// bit 0 of each byte is the motor stall.
bool wheelStalled(const UndockInputs& inputs);
bool stationary(const UndockInputs& inputs);

// ---------------------------------------------------------------------------
// Hardware effects
// ---------------------------------------------------------------------------

enum HardwareEffect {
  EFFECT_NONE = 0,
  EFFECT_SEND_CHARGE_RELEASE,  // comInt(68, 0)
  EFFECT_ENABLE_MOTORS,
  EFFECT_DISABLE_MOTORS
};

// A step emits at most this many effects. Two exist so the release-and-enable
// transition can be applied under one robot lock without a snapshot in between.
const unsigned int kMaxEffectsPerStep = 2;

struct UndockOutputs {
  UndockOutputs();

  // Commanded reverse velocity in m/s, always <= 0. There is no angular field:
  // the SBC never rotates.
  double commanded_linear;

  // Edge-triggered. The caller applies effects[0..effect_count) exactly once for
  // each new effect_seq and never re-applies a sequence it has already seen, so a
  // phase lasting a second cannot call enableMotors() twenty times.
  HardwareEffect effects[kMaxEffectsPerStep];
  unsigned int effect_count;
  unsigned long effect_seq;

  UndockPhase phase;
  bool phase_changed;
  bool active;
  bool finished;
  bool success;
  int code;
  std::string message;

  // Progress, all in metres/radians.
  double reverse_progress_m;
  double lateral_error_m;
  double heading_error_rad;
  int release_attempts;
  double minimum_rear_range_m;
};

// ---------------------------------------------------------------------------
// DockObserver
// ---------------------------------------------------------------------------

class DockObserver {
 public:
  DockObserver();

  // Advances the observational transitions. Does nothing while DEPARTING: the
  // controller owns the state for that interval.
  void update(const UndockInputs& inputs, const UndockHardwareProfile& profile);

  DockPhysicalState state() const { return state_; }

  // Called by UndockController.
  void beginDeparture(const UndockInputs& inputs);
  void confirmClear(const UndockInputs& inputs);
  void faultStop();

  // Explicit recovery from FAULT_STOPPED. Refused while the underlying condition
  // persists. A charger release never clears a fault.
  bool reset(const UndockInputs& inputs, const UndockHardwareProfile& profile,
             std::string* error);

  // Auto-docking stays inhibited until BOTH the configured time has passed AND
  // the robot is the configured distance from where the departure began. Returns
  // false when no undock has completed in this process.
  bool redockInhibited(const UndockInputs& inputs,
                       const UndockHardwareProfile& profile) const;
  long redockInhibitRemainingMs(const UndockInputs& inputs,
                                const UndockHardwareProfile& profile) const;

 private:
  DockPhysicalState state_;
  int charge_debounce_;
  bool debounced_charging_;
  bool previous_debounced_charging_;
  bool seen_any_sample_;
  // A missed raw charge edge may be repaired only by sustained physical
  // charger voltage. State 2/3 alone is never dock evidence.
  long long dock_power_stationary_since_ms_;

  bool undock_completed_;
  long long undock_completed_ms_;
  double departure_x_mm_;
  double departure_y_mm_;
  // Set by a rising charge edge that followed a confirmed zero. Without it, no
  // amount of stale charge level can move the observer back to DOCKED.
  bool redock_pending_;
};

// ---------------------------------------------------------------------------
// UndockController
// ---------------------------------------------------------------------------

class UndockController {
 public:
  UndockController();

  // Validates the goal and every precondition, and either starts the operation or
  // rejects it synchronously with a code and a protocol-safe message token.
  bool request(const std::string& request_id, const UndockConfig& config,
               const UndockInputs& inputs, const UndockHardwareProfile& profile,
               DockObserver* observer, int* code, std::string* message);

  // Cancellation is keyed on the TARGET operation's id, not a new request id, and
  // does not report success until the operation is terminal with velocity zero and
  // motors disabled. Returns false when the id does not match a running operation.
  bool cancel(const std::string& target_request_id);

  // Session loss, shutdown, or the lease watchdog firing. The caller is
  // responsible for having already stopped the base and disabled the motors.
  void invalidate(const char* reason, DockObserver* observer);

  UndockOutputs step(const UndockInputs& inputs);

  bool active() const { return active_; }
  const std::string& requestId() const { return request_id_; }
  UndockPhase phase() const { return phase_; }
  int releaseAttempts() const { return release_attempts_; }

  // Age of the command lease. The independent watchdog thread stops the base and
  // disables the motors once this exceeds profile.lease_max_age_ms, because an
  // overall timeout inside step() cannot help if step() stops being called.
  long leaseAgeMs(long long now_ms) const;
  bool leaseValid() const { return active_ && lease_set_; }

 private:
  void enterPhase(UndockPhase phase, long long now_ms);
  void beginStopping(int code, const char* message, long long now_ms);
  void enterRecovery(int failure_code, const char* reason, long long now_ms);
  void queueEffect(UndockOutputs* out, HardwareEffect effect);
  void resetMotionReference(const UndockInputs& inputs);
  void captureDeparturePose(const UndockInputs& inputs);
  // Second reference, taken when the robot comes free of the dock. Distance is
  // always measured from the departure pose; deviation is measured from here.
  void captureGuardPose(const UndockInputs& inputs);
  void computeProgress(const UndockInputs& inputs, UndockOutputs* out) const;
  bool safetyTripped(const UndockInputs& inputs, UndockOutputs* out);
  void runPhase(const UndockInputs& inputs, UndockOutputs* out);

  bool active_;
  std::string request_id_;
  UndockConfig config_;
  UndockHardwareProfile profile_;
  DockObserver* observer_;

  UndockPhase phase_;
  long long started_ms_;
  long long phase_entered_ms_;
  bool lease_set_;
  long long last_step_ms_;
  unsigned long effect_seq_;

  bool cancel_requested_;
  int stop_code_;
  std::string stop_message_;

  int release_attempts_;
  int recovery_failure_code_;
  long long last_charge_resend_ms_;
  // One-shot guards, so a phase that lasts a second issues its effect once.
  bool prepare_disable_issued_;
  bool release_sent_;
  bool enable_issued_;
  bool handoff_enable_issued_;
  bool stop_disable_issued_;

  bool departure_captured_;
  double start_x_mm_;
  double start_y_mm_;
  double start_yaw_rad_;
  bool guard_captured_;
  double guard_x_mm_;
  double guard_y_mm_;
  double guard_yaw_rad_;
  double effective_clearance_m_;

  double motion_reference_odometer_m_;
  long long motion_reference_ms_;

  int stationary_cycles_;
};

}  // namespace patrolbot

#endif  // PATROLBOT_UNDOCK_HPP
