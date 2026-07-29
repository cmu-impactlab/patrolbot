#include "undock.hpp"

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace patrolbot {
namespace {

const double kStationaryLinear = 0.005;   // m/s
const double kStationaryAngular = 0.01;   // rad/s
// Bounded wait for a hardware acknowledgement inside one phase.
const long kPhaseAckTimeoutMs = 3000;
// Reverse progress that must never go positive: the robot moving forward while
// undocking is a fault, not slow progress.
const double kForwardMotionToleranceM = 0.02;
// Head-room above the hard clearance cap before the distance guard fires.
const double kDistanceCapSlackM = 0.05;
// Consecutive stationary samples required before declaring the handoff safe.
const int kHandoffStationaryCycles = 2;
// One main-loop period of slack, so a commissioned release_observe_ms of exactly
// one cycle is not declared failed by sampling jitter alone.
const long kTickToleranceMs = 60;

double wrapToPi(double radians) {
  while (radians > M_PI) radians -= 2.0 * M_PI;
  while (radians < -M_PI) radians += 2.0 * M_PI;
  return radians;
}

}  // namespace

// ---------------------------------------------------------------------------
// Plain structures
// ---------------------------------------------------------------------------

RearSonarStatus::RearSonarStatus()
    : configured(false), all_required_sensors_fresh(false),
      fresh_sensor_count(0), minimum_obstacle_range_m(0.0),
      near_field_unreliable(false) {}

bool RearSonarStatus::usable() const {
  return configured && all_required_sensors_fresh && !near_field_unreliable;
}

UndockHardwareProfile::UndockHardwareProfile()
    // Every physically validated fact starts false. A production undock is
    // refused until a commissioning file supplies real values.
    : bumper_mapping_valid(false),
      front_bumper_mask(0x7e00),
      rear_bumper_mask(0x007e),
      rear_sonar_mapping_valid(false),
      rear_near_field_floor_m(0.15),
      rear_stop_range_m(0.25),
      rear_sonar_fresh_ms(500),
      estop_semantics_valid(false),
      // bit 2 general_pdb_failure, bit 8 encoder_failure,
      // bit 9 critical_driving_fault, bit 15 jodrive_safe_mode.
      critical_fault_mask(0x8304),
      // Additionally blocks starting: bit 4 gyro_fault,
      // bit 7 encoder_degraded_warning. low_battery and high_temperature are
      // deliberately excluded: a docked robot legitimately reports them.
      precheck_blocking_fault_mask(0x8394),
      release_observe_ms(300),
      min_motion_m(0.01),
      motion_check_ms(1500),
      max_clearance_m(0.70),
      max_speed_mps(0.10),
      max_timeout_s(60.0),
      max_lateral_drift_m(0.06),
      max_departure_yaw_rad(0.20),
      pose_guard_start_m(0.25),
      braking_margin_m(0.10),
      sonar_uncertainty_m(0.05),
      charge_resend_min_ms(400),
      max_release_attempts(3),
      debounce_cycles(3),
      lease_max_age_ms(250),
      clearance_extension_m(0.05),
      redock_dwell_ms(2000),
      validation_max_clearance_m(0.10),
      validation_max_speed_mps(0.05),
      redock_inhibit_ms(60000),
      redock_inhibit_m(0.50) {}

bool UndockHardwareProfile::commissioned() const {
  return bumper_mapping_valid && rear_sonar_mapping_valid &&
         estop_semantics_valid && front_bumper_mask != 0 &&
         rear_bumper_mask != 0 && !rear_sonar_indices.empty();
}

UndockConfig::UndockConfig()
    : clearance_m(0.0), speed_mps(0.0), timeout_s(0.0),
      validation_mode(false) {}

UndockInputs::UndockInputs()
    : now_ms(0), charge_state(-1), motors_enabled(false),
      // Fail closed: a snapshot that was never populated must not read as safe.
      estop_pressed(true), fault_flags(0), stall_value(0),
      enc_x_mm(0.0), enc_y_mm(0.0), enc_yaw_deg(0.0), odometer_m(0.0),
      linear(0.0), angular(0.0) {}

UndockOutputs::UndockOutputs()
    : commanded_linear(0.0), effect_count(0), effect_seq(0),
      phase(UNDOCK_PHASE_IDLE), phase_changed(false), active(false),
      finished(false), success(false), code(UNDOCK_OK),
      reverse_progress_m(0.0), lateral_error_m(0.0), heading_error_rad(0.0),
      release_attempts(0), minimum_rear_range_m(0.0) {
  for (unsigned int i = 0; i < kMaxEffectsPerStep; ++i) {
    effects[i] = EFFECT_NONE;
  }
}

// ---------------------------------------------------------------------------
// Snapshot decoding
// ---------------------------------------------------------------------------

bool frontContactPressed(const UndockInputs& inputs,
                         const UndockHardwareProfile& profile) {
  if (!profile.bumper_mapping_valid) return false;
  return (inputs.stall_value & profile.front_bumper_mask) != 0;
}

bool rearBumperPressed(const UndockInputs& inputs,
                       const UndockHardwareProfile& profile) {
  if (!profile.bumper_mapping_valid) return false;
  return (inputs.stall_value & profile.rear_bumper_mask) != 0;
}

bool wheelStalled(const UndockInputs& inputs) {
  // ARIA packs the left wheel in the high byte and the right in the low byte;
  // bit 0 of each byte is the motor stall. The remaining bits are bumpers.
  const bool left = (inputs.stall_value & 0x0100u) != 0;
  const bool right = (inputs.stall_value & 0x0001u) != 0;
  return left || right;
}

bool stationary(const UndockInputs& inputs) {
  return std::fabs(inputs.linear) <= kStationaryLinear &&
         std::fabs(inputs.angular) <= kStationaryAngular;
}

// ---------------------------------------------------------------------------
// DockObserver
// ---------------------------------------------------------------------------

DockObserver::DockObserver()
    : state_(DOCK_UNKNOWN), charge_debounce_(0), debounced_charging_(false),
      previous_debounced_charging_(false), seen_any_sample_(false),
      charging_stationary_since_ms_(-1), undock_completed_(false),
      undock_completed_ms_(-1), departure_x_mm_(0.0), departure_y_mm_(0.0),
      redock_pending_(false) {}

void DockObserver::update(const UndockInputs& inputs,
                          const UndockHardwareProfile& profile) {
  // The debounce bookkeeping runs in every state, including DEPARTING, so the
  // rising-edge history stays current across an undock.
  const bool charging_sample = inputs.charge_state > 0;
  if (charging_sample == debounced_charging_ && seen_any_sample_) {
    charge_debounce_ = 0;
  } else {
    ++charge_debounce_;
    if (charge_debounce_ >= profile.debounce_cycles || !seen_any_sample_) {
      previous_debounced_charging_ = debounced_charging_;
      debounced_charging_ = charging_sample;
      charge_debounce_ = 0;
      seen_any_sample_ = true;
      if (debounced_charging_ && !previous_debounced_charging_) {
        redock_pending_ = true;
        charging_stationary_since_ms_ = inputs.now_ms;
      }
    }
  }

  // The controller owns the state while it owns motion.
  if (state_ == DOCK_DEPARTING) return;
  // Only an explicit reset leaves a fault.
  if (state_ == DOCK_FAULT_STOPPED) return;

  const bool contact = frontContactPressed(inputs, profile);
  const bool still = stationary(inputs);

  if (!debounced_charging_ || !still) {
    redock_pending_ = false;
    charging_stationary_since_ms_ = -1;
  }

  switch (state_) {
    case DOCK_UNKNOWN:
      if (debounced_charging_ || contact) {
        state_ = DOCK_DOCKED_CONFIRMED;
      } else if (still) {
        // Explicit clear verification. Unreachable while charging or in
        // contact, so it cannot manufacture a false clearance.
        state_ = DOCK_CLEAR_CONFIRMED;
      }
      break;

    case DOCK_DOCKED_CONFIRMED:
      // A robot whose charging has genuinely stopped with no contact evidence is
      // no longer docked. This is the same observational test as from UNKNOWN,
      // and matches the permission the drive gate granted before this change.
      if (!debounced_charging_ && !contact && still) {
        state_ = DOCK_CLEAR_CONFIRMED;
        redock_pending_ = false;
      }
      break;

    case DOCK_CLEAR_CONFIRMED:
      // A stale charge level, however long it is held, is never evidence of
      // re-docking. Only a rising edge after a confirmed zero counts, and even
      // then it needs contact evidence or a stationary dwell.
      if (redock_pending_ && debounced_charging_ && still) {
        if (contact) {
          state_ = DOCK_DOCKED_CONFIRMED;
          redock_pending_ = false;
        } else if (!redockInhibited(inputs, profile) &&
                   charging_stationary_since_ms_ >= 0 &&
                   inputs.now_ms - charging_stationary_since_ms_ >=
                       profile.redock_dwell_ms) {
          // No contact bit available. A long stationary dwell is accepted only
          // once the robot is no longer inside its post-undock inhibition, so a
          // stale report seconds after a proven departure cannot revoke it.
          state_ = DOCK_DOCKED_CONFIRMED;
          redock_pending_ = false;
        }
      }
      break;

    default:
      break;
  }
}

void DockObserver::beginDeparture(const UndockInputs& inputs) {
  state_ = DOCK_DEPARTING;
  departure_x_mm_ = inputs.enc_x_mm;
  departure_y_mm_ = inputs.enc_y_mm;
  undock_completed_ = false;
  undock_completed_ms_ = -1;
}

void DockObserver::confirmClear(const UndockInputs& inputs) {
  state_ = DOCK_CLEAR_CONFIRMED;
  undock_completed_ = true;
  undock_completed_ms_ = inputs.now_ms;
  // Any edge observed while the controller owned the state belongs to the
  // departure, not to a new docking event.
  redock_pending_ = false;
  charging_stationary_since_ms_ = -1;
}

void DockObserver::faultStop() {
  state_ = DOCK_FAULT_STOPPED;
  redock_pending_ = false;
  charging_stationary_since_ms_ = -1;
}

bool DockObserver::reset(const UndockInputs& inputs,
                         const UndockHardwareProfile& profile,
                         std::string* error) {
  if (state_ != DOCK_FAULT_STOPPED) {
    if (error != NULL) *error = "not_faulted";
    return false;
  }
  if (inputs.estop_pressed) {
    if (error != NULL) *error = "estop_asserted";
    return false;
  }
  if ((inputs.fault_flags & profile.precheck_blocking_fault_mask) != 0) {
    if (error != NULL) *error = "hardware_fault";
    return false;
  }
  if (!stationary(inputs)) {
    if (error != NULL) *error = "robot_not_stationary";
    return false;
  }
  if (rearBumperPressed(inputs, profile)) {
    if (error != NULL) *error = "rear_bumper_pressed";
    return false;
  }
  state_ = DOCK_UNKNOWN;
  if (error != NULL) error->clear();
  return true;
}

bool DockObserver::redockInhibited(const UndockInputs& inputs,
                                   const UndockHardwareProfile& profile) const {
  if (!undock_completed_) return false;
  const bool time_elapsed =
      inputs.now_ms - undock_completed_ms_ >= profile.redock_inhibit_ms;
  const double dx = (inputs.enc_x_mm - departure_x_mm_) / 1000.0;
  const double dy = (inputs.enc_y_mm - departure_y_mm_) / 1000.0;
  const bool far_enough =
      std::sqrt(dx * dx + dy * dy) >= profile.redock_inhibit_m;
  // Inhibited until BOTH conditions hold.
  return !(time_elapsed && far_enough);
}

long DockObserver::redockInhibitRemainingMs(
    const UndockInputs& inputs, const UndockHardwareProfile& profile) const {
  if (!undock_completed_) return 0;
  const long long remaining =
      profile.redock_inhibit_ms - (inputs.now_ms - undock_completed_ms_);
  if (remaining <= 0) return 0;
  return static_cast<long>(remaining);
}

// ---------------------------------------------------------------------------
// UndockController
// ---------------------------------------------------------------------------

UndockController::UndockController()
    : active_(false), observer_(NULL), phase_(UNDOCK_PHASE_IDLE),
      started_ms_(0), phase_entered_ms_(0), lease_set_(false), last_step_ms_(0),
      effect_seq_(0), cancel_requested_(false), stop_code_(UNDOCK_OK),
      release_attempts_(0), recovery_failure_code_(UNDOCK_RELEASE_FAILED),
      last_charge_resend_ms_(0), prepare_disable_issued_(false),
      release_sent_(false), enable_issued_(false), handoff_enable_issued_(false),
      stop_disable_issued_(false), departure_captured_(false),
      start_x_mm_(0.0), start_y_mm_(0.0), start_yaw_rad_(0.0),
      effective_clearance_m_(0.0), motion_reference_odometer_m_(0.0),
      motion_reference_ms_(0), stationary_cycles_(0) {}

bool UndockController::request(const std::string& request_id,
                               const UndockConfig& config,
                               const UndockInputs& inputs,
                               const UndockHardwareProfile& profile,
                               DockObserver* observer, int* code,
                               std::string* message) {
  const int kNoCode = -1;
  int local_code = kNoCode;
  std::string local_message;

  if (active_) {
    local_code = UNDOCK_BUSY;
    local_message = "another_undock_is_active";
  } else if (observer == NULL) {
    local_code = UNDOCK_INVALID_REQUEST;
    local_message = "no_dock_observer";
  } else {
    // Goal bounds. Rejected, never silently clamped.
    const double max_clearance = config.validation_mode
                                     ? profile.validation_max_clearance_m
                                     : profile.max_clearance_m;
    const double max_speed = config.validation_mode
                                 ? profile.validation_max_speed_mps
                                 : profile.max_speed_mps;
    if (!(config.clearance_m > 0.0) || config.clearance_m > max_clearance) {
      local_code = UNDOCK_INVALID_REQUEST;
      local_message = "clearance_out_of_range";
    } else if (!(config.speed_mps > 0.0) || config.speed_mps > max_speed) {
      local_code = UNDOCK_INVALID_REQUEST;
      local_message = "speed_out_of_range";
    } else if (!(config.timeout_s > 0.0) ||
               config.timeout_s > profile.max_timeout_s) {
      local_code = UNDOCK_INVALID_REQUEST;
      local_message = "timeout_out_of_range";
    } else if (!config.validation_mode && !profile.commissioned()) {
      local_code = UNDOCK_HARDWARE_PROFILE_UNCOMMISSIONED;
      local_message = "hardware_profile_uncommissioned";
    } else if (observer->state() != DOCK_DOCKED_CONFIRMED) {
      local_code = UNDOCK_PRECONDITION_FAILED;
      local_message = "dock_state_is_not_docked_confirmed";
    } else if (inputs.estop_pressed) {
      local_code = UNDOCK_PRECONDITION_FAILED;
      local_message = "estop_asserted";
    } else if ((inputs.fault_flags & profile.precheck_blocking_fault_mask) != 0) {
      local_code = UNDOCK_PRECONDITION_FAILED;
      local_message = "hardware_fault";
    } else if (!stationary(inputs)) {
      local_code = UNDOCK_PRECONDITION_FAILED;
      local_message = "robot_not_stationary";
    } else if (rearBumperPressed(inputs, profile)) {
      local_code = UNDOCK_PRECONDITION_FAILED;
      local_message = "rear_bumper_pressed";
    } else if (!inputs.rear.usable()) {
      local_code = UNDOCK_PRECONDITION_FAILED;
      local_message = "rear_sonar_unusable";
    } else if (inputs.rear.minimum_obstacle_range_m <
               config.clearance_m + profile.braking_margin_m +
                   profile.sonar_uncertainty_m) {
      // Not merely "no obstacle within the stop range": the requested reverse
      // has to fit, with braking and measurement margin.
      local_code = UNDOCK_PRECONDITION_FAILED;
      local_message = "rear_clearance_insufficient_for_requested_distance";
    }
  }

  if (local_code != kNoCode) {
    if (code != NULL) *code = local_code;
    if (message != NULL) *message = local_message;
    return false;
  }

  active_ = true;
  request_id_ = request_id;
  config_ = config;
  profile_ = profile;
  observer_ = observer;
  phase_ = UNDOCK_PHASE_PREPARING;
  started_ms_ = inputs.now_ms;
  phase_entered_ms_ = inputs.now_ms;
  last_step_ms_ = inputs.now_ms;
  lease_set_ = true;
  cancel_requested_ = false;
  stop_code_ = UNDOCK_OK;
  stop_message_.clear();
  release_attempts_ = 0;
  recovery_failure_code_ = UNDOCK_RELEASE_FAILED;
  last_charge_resend_ms_ = inputs.now_ms - profile_.charge_resend_min_ms;
  prepare_disable_issued_ = false;
  release_sent_ = false;
  enable_issued_ = false;
  handoff_enable_issued_ = false;
  stop_disable_issued_ = false;
  departure_captured_ = false;
  guard_captured_ = false;
  effective_clearance_m_ = config_.clearance_m;
  stationary_cycles_ = 0;
  observer_->beginDeparture(inputs);

  if (code != NULL) *code = UNDOCK_OK;
  if (message != NULL) *message = "accepted";
  return true;
}

bool UndockController::cancel(const std::string& target_request_id) {
  if (!active_ || target_request_id != request_id_) return false;
  cancel_requested_ = true;
  // Deliberately NOT terminal here. The operation reports CANCELED only once
  // step() has confirmed zero velocity and disabled the motors.
  return true;
}

void UndockController::invalidate(const char* reason, DockObserver* observer) {
  if (!active_) return;
  active_ = false;
  lease_set_ = false;
  phase_ = UNDOCK_PHASE_IDLE;
  stop_code_ = UNDOCK_SESSION_CHANGED;
  stop_message_ = (reason != NULL) ? reason : "invalidated";
  if (observer != NULL) observer->faultStop();
}

long UndockController::leaseAgeMs(long long now_ms) const {
  if (!lease_set_) return 0;
  const long long age = now_ms - last_step_ms_;
  if (age <= 0) return 0;
  return static_cast<long>(age);
}

void UndockController::enterPhase(UndockPhase phase, long long now_ms) {
  phase_ = phase;
  phase_entered_ms_ = now_ms;
}

void UndockController::beginStopping(int code, const char* message,
                                     long long now_ms) {
  if (phase_ == UNDOCK_PHASE_STOPPING) return;
  stop_code_ = code;
  stop_message_ = message;
  stop_disable_issued_ = false;
  enterPhase(UNDOCK_PHASE_STOPPING, now_ms);
}

void UndockController::enterRecovery(int failure_code, const char* reason,
                                     long long now_ms) {
  ++release_attempts_;
  recovery_failure_code_ = failure_code;
  if (release_attempts_ > profile_.max_release_attempts) {
    beginStopping(failure_code, reason, now_ms);
    return;
  }
  // The same bounded operation resumes: the captured departure pose, the
  // distance budget and the overall deadline all carry over.
  release_sent_ = false;
  enable_issued_ = false;
  enterPhase(UNDOCK_PHASE_RELEASING, now_ms);
}

void UndockController::queueEffect(UndockOutputs* out, HardwareEffect effect) {
  if (out->effect_count >= kMaxEffectsPerStep) return;
  if (out->effect_count == 0) {
    // A new sequence number marks these effects as not-yet-applied. The caller
    // applies each sequence exactly once, so a phase lasting a second cannot
    // re-issue enableMotors() twenty times.
    out->effect_seq = ++effect_seq_;
  }
  out->effects[out->effect_count] = effect;
  ++out->effect_count;
}

void UndockController::resetMotionReference(const UndockInputs& inputs) {
  motion_reference_odometer_m_ = inputs.odometer_m;
  motion_reference_ms_ = inputs.now_ms;
}

void UndockController::captureDeparturePose(const UndockInputs& inputs) {
  start_x_mm_ = inputs.enc_x_mm;
  start_y_mm_ = inputs.enc_y_mm;
  start_yaw_rad_ = inputs.enc_yaw_deg * M_PI / 180.0;
  departure_captured_ = true;
}

void UndockController::captureGuardPose(const UndockInputs& inputs) {
  guard_x_mm_ = inputs.enc_x_mm;
  guard_y_mm_ = inputs.enc_y_mm;
  guard_yaw_rad_ = inputs.enc_yaw_deg * M_PI / 180.0;
  guard_captured_ = true;
}

void UndockController::computeProgress(const UndockInputs& inputs,
                                       UndockOutputs* out) const {
  if (!departure_captured_) {
    out->reverse_progress_m = 0.0;
    out->lateral_error_m = 0.0;
    out->heading_error_rad = 0.0;
    return;
  }
  // ArPose reports millimetres and degrees. The conversion happens here, once.
  const double dx = (inputs.enc_x_mm - start_x_mm_) / 1000.0;
  const double dy = (inputs.enc_y_mm - start_y_mm_) / 1000.0;
  const double cos_yaw = std::cos(start_yaw_rad_);
  const double sin_yaw = std::sin(start_yaw_rad_);
  // Signed progress along the reverse of the departure heading. Euclidean
  // distance would count movement in any direction, including sideways slip.
  // This is always measured from the dock, because that is what clearance
  // means.
  out->reverse_progress_m = -(dx * cos_yaw + dy * sin_yaw);

  if (!guard_captured_) {
    // Still mechanically captured. Measured on this dock 2026-07-29: the robot
    // is twisted about 12 degrees, confirmed independently by AMCL, while the
    // charger is still engaged at 0.147 m. That rotation is imposed by the
    // fixture, so reporting it as controller deviation aborted a departure that
    // was behaving exactly as the hardware forces it to.
    out->lateral_error_m = 0.0;
    out->heading_error_rad = 0.0;
    return;
  }
  // Free-running reference: deviation is what the robot has done since it came
  // off the dock, which is the only part the controller is responsible for.
  const double gx = (inputs.enc_x_mm - guard_x_mm_) / 1000.0;
  const double gy = (inputs.enc_y_mm - guard_y_mm_) / 1000.0;
  const double guard_cos = std::cos(guard_yaw_rad_);
  const double guard_sin = std::sin(guard_yaw_rad_);
  out->lateral_error_m = std::fabs(-gx * guard_sin + gy * guard_cos);
  out->heading_error_rad = std::fabs(
      wrapToPi(inputs.enc_yaw_deg * M_PI / 180.0 - guard_yaw_rad_));
}

bool UndockController::safetyTripped(const UndockInputs& inputs,
                                     UndockOutputs* out) {
  (void)out;
  if (phase_ == UNDOCK_PHASE_STOPPING) return false;
  if (cancel_requested_) {
    beginStopping(UNDOCK_CANCELED, "canceled", inputs.now_ms);
    return true;
  }
  if (inputs.estop_pressed) {
    beginStopping(UNDOCK_SAFETY_FAULT, "estop_asserted", inputs.now_ms);
    return true;
  }
  if ((inputs.fault_flags & profile_.critical_fault_mask) != 0) {
    beginStopping(UNDOCK_SAFETY_FAULT, "critical_hardware_fault",
                  inputs.now_ms);
    return true;
  }
  if (rearBumperPressed(inputs, profile_)) {
    beginStopping(UNDOCK_SAFETY_FAULT, "rear_bumper_pressed", inputs.now_ms);
    return true;
  }
  // A wheel stall while reversing is a hard stop, never a release retry: it is
  // a different failure from the controller re-latching the charger.
  if (phase_ == UNDOCK_PHASE_BACKING_OUT && wheelStalled(inputs)) {
    beginStopping(UNDOCK_SAFETY_FAULT, "wheel_stall", inputs.now_ms);
    return true;
  }
  if (inputs.now_ms - started_ms_ >
      static_cast<long long>(config_.timeout_s * 1000.0)) {
    beginStopping(UNDOCK_TIMEOUT, "operation_timeout", inputs.now_ms);
    return true;
  }
  return false;
}

void UndockController::runPhase(const UndockInputs& inputs,
                                UndockOutputs* out) {
  out->commanded_linear = 0.0;
  const long long now = inputs.now_ms;

  switch (phase_) {
    case UNDOCK_PHASE_PREPARING: {
      if (!prepare_disable_issued_) {
        queueEffect(out, EFFECT_DISABLE_MOTORS);
        prepare_disable_issued_ = true;
      }
      if (!inputs.motors_enabled && stationary(inputs)) {
        release_sent_ = false;
        enterPhase(UNDOCK_PHASE_RELEASING, now);
      } else if (now - phase_entered_ms_ > kPhaseAckTimeoutMs) {
        beginStopping(UNDOCK_MOTOR_STATE_FAILED, "motors_not_disabled", now);
      }
      break;
    }

    case UNDOCK_PHASE_RELEASING: {
      if (!release_sent_) {
        queueEffect(out, EFFECT_SEND_CHARGE_RELEASE);
        last_charge_resend_ms_ = now;
        release_sent_ = true;
        break;
      }
      // A falling edge or a single valid zero sample is enough. Requiring a
      // multi-sample dwell would miss a release that only lasts one cycle.
      if (inputs.charge_state == 0) {
        enable_issued_ = false;
        enterPhase(UNDOCK_PHASE_ENABLING_MOTORS, now);
      } else if (now - phase_entered_ms_ >
                 profile_.release_observe_ms + kTickToleranceMs) {
        ++release_attempts_;
        if (release_attempts_ > profile_.max_release_attempts) {
          beginStopping(UNDOCK_RELEASE_FAILED, "charge_release_not_observed",
                        now);
        } else {
          release_sent_ = false;
          phase_entered_ms_ = now;
        }
      }
      break;
    }

    case UNDOCK_PHASE_ENABLING_MOTORS: {
      if (!enable_issued_) {
        // Both effects are applied under one robot lock, with no snapshot in
        // between, and never alongside a nonzero velocity.
        queueEffect(out, EFFECT_SEND_CHARGE_RELEASE);
        queueEffect(out, EFFECT_ENABLE_MOTORS);
        last_charge_resend_ms_ = now;
        enable_issued_ = true;
        break;
      }
      if (inputs.motors_enabled) {
        if (!departure_captured_) captureDeparturePose(inputs);
        resetMotionReference(inputs);
        enterPhase(UNDOCK_PHASE_BACKING_OUT, now);
      } else if (now - phase_entered_ms_ > kPhaseAckTimeoutMs) {
        enterRecovery(UNDOCK_MOTOR_STATE_FAILED, "motors_not_enabled", now);
      }
      break;
    }

    case UNDOCK_PHASE_BACKING_OUT: {
      // Motion actually interrupted: stop, release, enable, resume.
      if (!inputs.motors_enabled) {
        enterRecovery(UNDOCK_RELEASE_FAILED, "motors_disabled_while_reversing",
                      now);
        break;
      }
      if (inputs.odometer_m - motion_reference_odometer_m_ >=
          profile_.min_motion_m) {
        resetMotionReference(inputs);
      } else if (now - motion_reference_ms_ > profile_.motion_check_ms) {
        enterRecovery(UNDOCK_MOTION_FAILED, "no_motion_after_reverse_command",
                      now);
        break;
      }

      if (!inputs.rear.usable() ||
          inputs.rear.minimum_obstacle_range_m < profile_.rear_stop_range_m) {
        beginStopping(UNDOCK_SENSOR_BLOCKED, "rear_path_blocked", now);
        break;
      }
      // The pose guard engages only after measured reverse progress proves the
      // robot is beyond the dock fixture. Charge=0 and a cleared front bumper
      // are not sufficient: on the real dock both happened before the fixture
      // finished imposing its roughly 12 degree rotation, which captured a
      // false free-running reference and aborted at 0.141 m.
      if (!guard_captured_ &&
          out->reverse_progress_m >= profile_.pose_guard_start_m) {
        captureGuardPose(inputs);
        computeProgress(inputs, out);
      }
      if (guard_captured_) {
        if (out->lateral_error_m > profile_.max_lateral_drift_m) {
          beginStopping(UNDOCK_MOTION_FAILED, "lateral_drift_exceeded", now);
          break;
        }
        if (out->heading_error_rad > profile_.max_departure_yaw_rad) {
          beginStopping(UNDOCK_MOTION_FAILED, "heading_error_exceeded", now);
          break;
        }
      }
      if (out->reverse_progress_m < -kForwardMotionToleranceM) {
        beginStopping(UNDOCK_MOTION_FAILED, "forward_motion_detected", now);
        break;
      }
      if (out->reverse_progress_m >
          profile_.max_clearance_m + kDistanceCapSlackM) {
        beginStopping(UNDOCK_MOTION_FAILED, "distance_cap_exceeded", now);
        break;
      }

      if (out->reverse_progress_m >= effective_clearance_m_) {
        out->commanded_linear = 0.0;
        stationary_cycles_ = 0;
        enterPhase(UNDOCK_PHASE_VERIFYING_CLEARANCE, now);
        break;
      }

      // Charging re-asserted but the reverse is healthy: keep going and nudge
      // the charger release at a bounded rate rather than stopping, which would
      // let the contacts settle back in.
      if (inputs.charge_state > 0 &&
          now - last_charge_resend_ms_ >= profile_.charge_resend_min_ms) {
        queueEffect(out, EFFECT_SEND_CHARGE_RELEASE);
        last_charge_resend_ms_ = now;
      }
      out->commanded_linear = -config_.speed_mps;
      break;
    }

    case UNDOCK_PHASE_VERIFYING_CLEARANCE: {
      if (!stationary(inputs)) {
        if (now - phase_entered_ms_ > kPhaseAckTimeoutMs) {
          beginStopping(UNDOCK_MOTION_FAILED, "did_not_stop_for_verification",
                        now);
        }
        break;
      }
      if (!inputs.rear.usable() ||
          inputs.rear.minimum_obstacle_range_m < profile_.rear_stop_range_m) {
        beginStopping(UNDOCK_SENSOR_BLOCKED, "rear_path_blocked", now);
        break;
      }
      if (frontContactPressed(inputs, profile_)) {
        // Still touching. Extend the target within the hard cap and go again.
        const double extended =
            effective_clearance_m_ + profile_.clearance_extension_m;
        if (extended > profile_.max_clearance_m) {
          beginStopping(UNDOCK_MOTION_FAILED, "front_contact_persists", now);
          break;
        }
        effective_clearance_m_ = extended;
        resetMotionReference(inputs);
        enterPhase(UNDOCK_PHASE_BACKING_OUT, now);
        break;
      }
      // Only meaningful once the free-running reference exists; before that
      // the reported deviation is the dock's doing, not the controller's.
      if (guard_captured_ &&
          (out->lateral_error_m > profile_.max_lateral_drift_m ||
           out->heading_error_rad > profile_.max_departure_yaw_rad)) {
        beginStopping(UNDOCK_MOTION_FAILED, "departure_pose_out_of_limits",
                      now);
        break;
      }
      if (out->reverse_progress_m < effective_clearance_m_) {
        resetMotionReference(inputs);
        enterPhase(UNDOCK_PHASE_BACKING_OUT, now);
        break;
      }
      stationary_cycles_ = 0;
      handoff_enable_issued_ = false;
      enterPhase(UNDOCK_PHASE_HANDOFF_READY, now);
      break;
    }

    case UNDOCK_PHASE_HANDOFF_READY: {
      // Success leaves the motors ENABLED at zero velocity so Nav2 can take
      // control. The generic drive gate refuses motion with motors disabled, so
      // handing off any other way would strand the robot.
      if (!inputs.motors_enabled) {
        if (!handoff_enable_issued_) {
          queueEffect(out, EFFECT_ENABLE_MOTORS);
          handoff_enable_issued_ = true;
          break;
        }
        if (now - phase_entered_ms_ > kPhaseAckTimeoutMs) {
          beginStopping(UNDOCK_MOTOR_STATE_FAILED, "motors_not_live_for_handoff",
                        now);
        }
        break;
      }
      if (!stationary(inputs)) {
        stationary_cycles_ = 0;
        if (now - phase_entered_ms_ > kPhaseAckTimeoutMs) {
          beginStopping(UNDOCK_MOTION_FAILED, "did_not_settle_for_handoff", now);
        }
        break;
      }
      if (++stationary_cycles_ < kHandoffStationaryCycles) break;

      observer_->confirmClear(inputs);
      out->finished = true;
      out->success = true;
      out->code = UNDOCK_OK;
      out->message = "undocked";
      active_ = false;
      lease_set_ = false;
      break;
    }

    case UNDOCK_PHASE_STOPPING: {
      if (!stop_disable_issued_) {
        queueEffect(out, EFFECT_DISABLE_MOTORS);
        stop_disable_issued_ = true;
      }
      const bool settled = stationary(inputs) && !inputs.motors_enabled;
      if (settled || now - phase_entered_ms_ > kPhaseAckTimeoutMs) {
        observer_->faultStop();
        out->finished = true;
        out->success = false;
        out->code = stop_code_;
        out->message = stop_message_.empty() ? "undock_failed" : stop_message_;
        active_ = false;
        lease_set_ = false;
      }
      break;
    }

    default:
      break;
  }
}

UndockOutputs UndockController::step(const UndockInputs& inputs) {
  UndockOutputs out;
  out.phase = phase_;
  out.active = active_;
  if (!active_) return out;

  last_step_ms_ = inputs.now_ms;
  lease_set_ = true;

  const UndockPhase entry_phase = phase_;
  computeProgress(inputs, &out);
  out.release_attempts = release_attempts_;
  out.minimum_rear_range_m = inputs.rear.minimum_obstacle_range_m;

  safetyTripped(inputs, &out);

  // Re-dispatch so a transition takes effect within the same 50 ms step, but
  // only while no hardware effect has been queued: that keeps every step to at
  // most one phase's worth of effects.
  int guard = 0;
  UndockPhase before = phase_;
  do {
    before = phase_;
    runPhase(inputs, &out);
    ++guard;
  } while (phase_ != before && out.effect_count == 0 && !out.finished &&
           guard < 4);

  computeProgress(inputs, &out);
  out.release_attempts = release_attempts_;
  out.phase = phase_;
  out.phase_changed = phase_ != entry_phase;
  out.active = active_;
  if (out.finished) out.phase_changed = true;
  return out;
}

}  // namespace patrolbot
