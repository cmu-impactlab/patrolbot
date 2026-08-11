#include <new>
#include <iterator>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <cstring>
#include <stdint.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <errno.h>
#include <cmath>
#include <deque>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "Aria.h"
#include "protocol.hpp"
#include "undock.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

const int kControlOk = 0;
const int kInvalidRequest = 1;
const int kPreconditionFailed = 3;
const int kHardwareRejected = 4;
const int kOperationTimeout = 5;
const int kSessionChanged = 6;
const int kStableCyclesRequired = 3;
const long kOperationTimeoutMs = 5000;
// PatrolBot third-generation user guide: uARCS CHARGE is command 68,
// argument 0 releases the autocharge mechanism. This legacy ARIA install does
// not define ArCommands::CHARGE (its generic enum gives 68 a simulator alias),
// so use the manual's hardware-protocol number explicitly. Argument 1, which
// halts the robot and deploys the charging mechanism, is never sent anywhere.
const unsigned char kUarcsChargeCommand = 68;
const char* kProfilePath = "/home/ros/patrolbot_hw_server/undock_profile.conf";
// Outgoing bytes are coalesced per iteration so a control reply no longer
// displaces a whole telemetry frame. Capped so back-pressure cannot grow it.
const std::size_t kMaxPendingWrite = 65536;

// Which producer is allowed to set a nonzero velocity. Replaces the implicit
// ownership the timed authorization used to imply.
enum MotionAuthority {
  AUTHORITY_NONE = 0,
  AUTHORITY_DRIVE,
  AUTHORITY_UNDOCK
};

enum ControlPhase {
  CONTROL_PHASE_NONE = 0,
  RELEASE_WAIT_MOTORS_DISABLED,
  RELEASE_WAIT_CHARGE_STATE,
  MOTOR_ENABLE_WAIT_ENABLED
};

struct HardwareSnapshot {
  double linear;
  double angular;
  int charge_state;
  bool motors_enabled;
  bool estop_pressed;
  unsigned int fault_flags;
  unsigned int stall_value;
};

struct PendingOperation {
  PendingOperation()
      : active(false), type(patrolbot::COMMAND_INVALID),
        phase(CONTROL_PHASE_NONE), stable_cycles(0) {}

  bool active;
  patrolbot::CommandType type;
  ControlPhase phase;
  std::string request_id;
  ArTime started;
  int stable_cycles;
};

// ArTime::mSecSince() is a long, which wraps after about 24 days on this
// 32-bit base. Accumulating small per-iteration deltas into a long long keeps
// the undock controller's monotonic clock correct for the life of the process.
class MonotonicClock {
 public:
  MonotonicClock() : now_ms_(0) { last_.setToNow(); }

  long long tick() {
    const long delta = last_.mSecSince();
    if (delta > 0) {
      now_ms_ += delta;
      last_.setToNow();
    }
    return now_ms_;
  }

  long long now() const { return now_ms_; }

 private:
  ArTime last_;
  long long now_ms_;
};

// The undock controller's own timeouts are useless if the main loop stops
// calling step(). This thread supervises the loop from outside it: if the
// command lease goes stale while the undock owns motion, it takes the robot
// lock itself, zeroes velocity and disables the motors.
class UndockLeaseWatchdog : public ArASyncTask {
 public:
  UndockLeaseWatchdog(ArRobot* robot, long max_age_ms)
      : robot_(robot), max_age_ms_(max_age_ms), armed_(false), tripped_(false),
        running_(true) {
    last_renew_.setToNow();
  }

  void arm() {
    mutex_.lock();
    armed_ = true;
    tripped_ = false;
    last_renew_.setToNow();
    mutex_.unlock();
  }

  void renew() {
    mutex_.lock();
    if (armed_) last_renew_.setToNow();
    mutex_.unlock();
  }

  void disarm() {
    mutex_.lock();
    armed_ = false;
    mutex_.unlock();
  }

  bool tripped() {
    mutex_.lock();
    const bool value = tripped_;
    mutex_.unlock();
    return value;
  }

  void clearTrip() {
    mutex_.lock();
    tripped_ = false;
    mutex_.unlock();
  }

  void requestStop() {
    mutex_.lock();
    running_ = false;
    armed_ = false;
    mutex_.unlock();
  }

  virtual void* runThread(void*) {
    while (true) {
      mutex_.lock();
      const bool running = running_;
      const bool expired =
          armed_ && !tripped_ && last_renew_.mSecSince() > max_age_ms_;
      if (expired) {
        tripped_ = true;
        armed_ = false;
      }
      mutex_.unlock();
      if (!running) break;
      if (expired) {
        robot_->lock();
        robot_->setVel(0);
        robot_->setRotVel(0);
        robot_->disableMotors();
        robot_->unlock();
        ArLog::log(ArLog::Terse,
                   "Undock command lease expired (>%ld ms without a control "
                   "step); base stopped and motors disabled.",
                   max_age_ms_);
      }
      ArUtil::sleep(50);
    }
    return NULL;
  }

 private:
  ArRobot* robot_;
  long max_age_ms_;
  ArMutex mutex_;
  bool armed_;
  ArTime last_renew_;
  bool tripped_;
  bool running_;
};

std::string trimmed(const std::string& value) {
  const std::string::size_type first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return "";
  const std::string::size_type last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

bool parseUnsigned(const std::string& text, unsigned int* result) {
  char* end = NULL;
  errno = 0;
  const unsigned long value = std::strtoul(text.c_str(), &end, 0);
  if (errno != 0 || end == text.c_str() || *end != '\0') return false;
  *result = static_cast<unsigned int>(value);
  return true;
}

// Reads the commissioned hardware profile. Anything absent keeps the built-in
// default, and every *_valid flag defaults to 0, so a missing or partial file
// leaves production undocking refused rather than silently permitted.
patrolbot::UndockHardwareProfile loadProfile(const char* path) {
  patrolbot::UndockHardwareProfile profile;
  std::ifstream file(path);
  if (!file.is_open()) {
    ArLog::log(ArLog::Terse,
               "No undock profile at %s; production UNDOCK stays refused.",
               path);
    return profile;
  }
  std::string line;
  while (std::getline(file, line)) {
    const std::string::size_type comment = line.find('#');
    if (comment != std::string::npos) line = line.substr(0, comment);
    const std::string::size_type equals = line.find('=');
    if (equals == std::string::npos) continue;
    const std::string key = trimmed(line.substr(0, equals));
    const std::string value = trimmed(line.substr(equals + 1));
    if (key.empty() || value.empty()) continue;

    if (key == "bumper_mapping_valid") {
      profile.bumper_mapping_valid = value != "0";
    } else if (key == "front_bumper_mask") {
      parseUnsigned(value, &profile.front_bumper_mask);
    } else if (key == "rear_bumper_mask") {
      parseUnsigned(value, &profile.rear_bumper_mask);
    } else if (key == "rear_sonar_mapping_valid") {
      profile.rear_sonar_mapping_valid = value != "0";
    } else if (key == "rear_sonar_indices") {
      profile.rear_sonar_indices.clear();
      std::stringstream stream(value);
      std::string token;
      while (std::getline(stream, token, ',')) {
        const std::string cleaned = trimmed(token);
        if (!cleaned.empty()) profile.rear_sonar_indices.push_back(std::atoi(cleaned.c_str()));
      }
    } else if (key == "rear_near_field_floor_m") {
      profile.rear_near_field_floor_m = std::atof(value.c_str());
    } else if (key == "rear_stop_range_m") {
      profile.rear_stop_range_m = std::atof(value.c_str());
    } else if (key == "rear_sonar_fresh_ms") {
      profile.rear_sonar_fresh_ms = std::atol(value.c_str());
    } else if (key == "estop_semantics_valid") {
      profile.estop_semantics_valid = value != "0";
    } else if (key == "critical_fault_mask") {
      parseUnsigned(value, &profile.critical_fault_mask);
    } else if (key == "precheck_blocking_fault_mask") {
      parseUnsigned(value, &profile.precheck_blocking_fault_mask);
    } else if (key == "release_observe_ms") {
      profile.release_observe_ms = std::atol(value.c_str());
    } else if (key == "min_motion_m") {
      profile.min_motion_m = std::atof(value.c_str());
    } else if (key == "motion_check_ms") {
      profile.motion_check_ms = std::atol(value.c_str());
    } else if (key == "max_clearance_m") {
      profile.max_clearance_m = std::atof(value.c_str());
    } else if (key == "max_speed_mps") {
      profile.max_speed_mps = std::atof(value.c_str());
    } else if (key == "max_timeout_s") {
      profile.max_timeout_s = std::atof(value.c_str());
    } else if (key == "max_lateral_drift_m") {
      profile.max_lateral_drift_m = std::atof(value.c_str());
    } else if (key == "max_departure_yaw_rad") {
      profile.max_departure_yaw_rad = std::atof(value.c_str());
    } else if (key == "pose_guard_start_m") {
      profile.pose_guard_start_m = std::atof(value.c_str());
    } else if (key == "braking_margin_m") {
      profile.braking_margin_m = std::atof(value.c_str());
    } else if (key == "sonar_uncertainty_m") {
      profile.sonar_uncertainty_m = std::atof(value.c_str());
    } else if (key == "charge_resend_min_ms") {
      profile.charge_resend_min_ms = std::atol(value.c_str());
    } else if (key == "max_release_attempts") {
      profile.max_release_attempts = std::atoi(value.c_str());
    } else if (key == "debounce_cycles") {
      profile.debounce_cycles = std::atoi(value.c_str());
    } else if (key == "lease_max_age_ms") {
      profile.lease_max_age_ms = std::atol(value.c_str());
    } else if (key == "clearance_extension_m") {
      profile.clearance_extension_m = std::atof(value.c_str());
    } else if (key == "redock_dwell_ms") {
      profile.redock_dwell_ms = std::atol(value.c_str());
    } else if (key == "redock_min_voltage_v") {
      profile.redock_min_voltage_v = std::atof(value.c_str());
    } else if (key == "validation_max_clearance_m") {
      profile.validation_max_clearance_m = std::atof(value.c_str());
    } else if (key == "validation_max_speed_mps") {
      profile.validation_max_speed_mps = std::atof(value.c_str());
    } else if (key == "redock_inhibit_ms") {
      profile.redock_inhibit_ms = std::atol(value.c_str());
    } else if (key == "redock_inhibit_m") {
      profile.redock_inhibit_m = std::atof(value.c_str());
    }
  }
  ArLog::log(ArLog::Normal,
             "Undock profile loaded from %s: commissioned=%d (bumpers=%d "
             "rear_sonar=%d estop=%d).",
             path, profile.commissioned() ? 1 : 0,
             profile.bumper_mapping_valid ? 1 : 0,
             profile.rear_sonar_mapping_valid ? 1 : 0,
             profile.estop_semantics_valid ? 1 : 0);
  return profile;
}

HardwareSnapshot readSnapshot(ArRobot* robot) {
  HardwareSnapshot state;
  robot->lock();
  state.linear = robot->getVel() / 1000.0;
  state.angular = robot->getRotVel() * M_PI / 180.0;
  state.charge_state = robot->getChargeState();
  state.motors_enabled = robot->areMotorsEnabled();
  state.estop_pressed = robot->isEStopPressed();
  state.fault_flags = robot->getFaultFlags();
  state.stall_value = robot->getStallValue();
  robot->unlock();
  return state;
}

// Reduces the commissioned rear transducers to one status. A sensor that is
// missing or stale is never reported as clear.
patrolbot::RearSonarStatus readRearSonar(
    ArRobot* robot, const patrolbot::UndockHardwareProfile& profile) {
  patrolbot::RearSonarStatus rear;
  if (!profile.rear_sonar_mapping_valid || profile.rear_sonar_indices.empty()) {
    return rear;
  }
  rear.configured = true;
  rear.minimum_obstacle_range_m = patrolbot::kSonarNoEchoRangeM;
  unsigned int fresh = 0;
  robot->lock();
  const int sonar_count = robot->getNumSonar();
  for (std::size_t i = 0; i < profile.rear_sonar_indices.size(); ++i) {
    const int index = profile.rear_sonar_indices[i];
    if (index < 0 || index >= sonar_count) continue;
    ArSensorReading* reading = robot->getSonarReading(index);
    if (reading == NULL) continue;
    if (reading->getTimeTaken().mSecSince() > profile.rear_sonar_fresh_ms) {
      continue;
    }
    ++fresh;
    const double range_m = reading->getRange() / 1000.0;
    // 4335 mm is the base's no-return sentinel: a fresh sensor seeing nothing.
    if (reading->getRange() >= 4335) continue;
    if (range_m < profile.rear_near_field_floor_m) {
      rear.near_field_unreliable = true;
    }
    if (range_m < rear.minimum_obstacle_range_m) {
      rear.minimum_obstacle_range_m = range_m;
    }
  }
  robot->unlock();
  rear.fresh_sensor_count = fresh;
  rear.all_required_sensors_fresh =
      fresh == profile.rear_sonar_indices.size();
  return rear;
}

patrolbot::UndockInputs readUndockInputs(
    ArRobot* robot, const patrolbot::UndockHardwareProfile& profile,
    long long now_ms) {
  patrolbot::UndockInputs inputs;
  inputs.now_ms = now_ms;
  robot->lock();
  inputs.charge_state = robot->getChargeState();
  inputs.battery_voltage_v = robot->getRealBatteryVoltageNow();
  inputs.motors_enabled = robot->areMotorsEnabled();
  inputs.estop_pressed = robot->isEStopPressed();
  inputs.fault_flags = robot->getFaultFlags();
  inputs.stall_value = robot->getStallValue();
  // Raw encoder pose: millimetres and degrees, converted exactly once inside
  // undock.cpp so the conversion cannot be duplicated or lost.
  const ArPose pose = robot->getEncoderPose();
  inputs.enc_x_mm = pose.getX();
  inputs.enc_y_mm = pose.getY();
  inputs.enc_yaw_deg = pose.getTh();
  inputs.odometer_m = robot->getOdometerDistanceMeters();
  inputs.linear = robot->getVel() / 1000.0;
  inputs.angular = robot->getRotVel() * M_PI / 180.0;
  robot->unlock();
  inputs.rear = readRearSonar(robot, profile);
  return inputs;
}

bool isStationary(const HardwareSnapshot& state) {
  return std::fabs(state.linear) <= 0.005 &&
         std::fabs(state.angular) <= 0.01;
}

// Applies at most one step's worth of effects, exactly once per sequence
// number. Both effects of a release-and-enable transition go out under one
// robot lock, with no snapshot in between.
void applyUndockEffects(ArRobot* robot, const patrolbot::UndockOutputs& out,
                        unsigned long* applied_seq) {
  if (out.effect_count == 0) return;
  if (out.effect_seq == *applied_seq) return;
  *applied_seq = out.effect_seq;
  robot->lock();
  for (unsigned int i = 0; i < out.effect_count; ++i) {
    switch (out.effects[i]) {
      case patrolbot::EFFECT_SEND_CHARGE_RELEASE:
        robot->comInt(kUarcsChargeCommand, 0);
        break;
      case patrolbot::EFFECT_ENABLE_MOTORS:
        robot->enableMotors();
        break;
      case patrolbot::EFFECT_DISABLE_MOTORS:
        robot->disableMotors();
        break;
      default:
        break;
    }
  }
  robot->unlock();
}

void finishOperation(PendingOperation* pending, bool success, int code,
                     const std::string& message,
                     const HardwareSnapshot& state,
                     std::deque<std::string>* responses,
                     patrolbot::RequestTracker* tracker,
                     bool enqueue_response = true) {
  const std::string result = patrolbot::formatResult(
      pending->type, pending->request_id, success, code, message,
      state.charge_state, state.motors_enabled, state.estop_pressed);
  tracker->finish(pending->type, pending->request_id, result);
  if (enqueue_response) responses->push_back(result);
  ArLog::log(success ? ArLog::Normal : ArLog::Terse,
             "%s %s completed: success=%d code=%d state=%d motors=%d estop=%d",
             patrolbot::commandName(pending->type).c_str(),
             pending->request_id.c_str(), success ? 1 : 0, code,
             state.charge_state, state.motors_enabled ? 1 : 0,
             state.estop_pressed ? 1 : 0);
  pending->active = false;
  pending->type = patrolbot::COMMAND_INVALID;
  pending->request_id.clear();
  pending->phase = CONTROL_PHASE_NONE;
  pending->stable_cycles = 0;
}

void rejectControl(const patrolbot::ParsedCommand& command, int code,
                   const std::string& message, const HardwareSnapshot& state,
                   std::deque<std::string>* responses,
                   patrolbot::RequestTracker* tracker) {
  PendingOperation rejected;
  rejected.active = true;
  rejected.type = command.type;
  rejected.request_id = command.request_id;
  responses->push_back(
      patrolbot::formatAck(command.type, command.request_id, false));
  finishOperation(&rejected, false, code, message, state, responses,
                  tracker);
}

// CHARGE_RELEASE and MOTOR_ENABLE are decoupled, zero-motion maintenance
// commands. CHARGE_RELEASE never enables motors and never authorizes motion.
// MOTOR_ENABLE never releases charging and is refused whenever the robot may
// still be connected to the dock. Neither of them can move the robot; that is
// exclusively the UNDOCK operation's job.
void startControl(const patrolbot::ParsedCommand& command, ArRobot* robot,
                  PendingOperation* pending,
                  std::deque<std::string>* responses,
                  patrolbot::RequestTracker* tracker,
                  patrolbot::DockObserver* observer,
                  const patrolbot::UndockInputs& inputs,
                  const patrolbot::UndockHardwareProfile& profile,
                  bool undock_active) {
  std::string completed_result;
  const patrolbot::RequestDisposition disposition = tracker->inspect(
      command.type, command.request_id, &completed_result);
  if (disposition == patrolbot::REQUEST_COMPLETED) {
    responses->push_back(
        patrolbot::formatAck(command.type, command.request_id, true));
    responses->push_back(completed_result);
    return;
  }
  if (disposition == patrolbot::REQUEST_DUPLICATE_PENDING) {
    responses->push_back(
        patrolbot::formatAck(command.type, command.request_id, true));
    return;
  }
  if (disposition == patrolbot::REQUEST_BUSY) {
    rejectControl(command, kPreconditionFailed, "another_operation_pending",
                  readSnapshot(robot), responses, tracker);
    return;
  }

  const HardwareSnapshot state = readSnapshot(robot);
  if (undock_active) {
    rejectControl(command, kPreconditionFailed, "undock_owns_motion", state,
                  responses, tracker);
    return;
  }
  if (!isStationary(state)) {
    rejectControl(command, kPreconditionFailed, "robot_not_stationary", state,
                  responses, tracker);
    return;
  }
  if ((state.fault_flags & profile.precheck_blocking_fault_mask) != 0) {
    rejectControl(command, kPreconditionFailed, "hardware_fault", state,
                  responses, tracker);
    return;
  }

  if (command.type == patrolbot::COMMAND_DOCK_RESET) {
    // Explicit recovery from FAULT_STOPPED. A charger release never clears a
    // fault: an encoder failure, a bumper fault or an E-stop is not resolved by
    // releasing the charger.
    std::string error;
    const bool ok = observer->reset(inputs, profile, &error);
    responses->push_back(
        patrolbot::formatAck(command.type, command.request_id, false));
    PendingOperation immediate;
    immediate.active = true;
    immediate.type = command.type;
    immediate.request_id = command.request_id;
    finishOperation(&immediate, ok, ok ? kControlOk : kPreconditionFailed,
                    ok ? "dock_state_reset" : error, state, responses, tracker);
    return;
  }

  if (command.type == patrolbot::COMMAND_CHARGE_RELEASE) {
    if (state.charge_state < 0) {
      rejectControl(command, kPreconditionFailed, "invalid_charge_state",
                    state, responses, tracker);
      return;
    }
    if (state.charge_state == 0 && state.motors_enabled) {
      rejectControl(command, kPreconditionFailed,
                    "already_released_with_motors_enabled", state,
                    responses, tracker);
      return;
    }
    responses->push_back(
        patrolbot::formatAck(command.type, command.request_id, false));
    if (state.charge_state == 0) {
      PendingOperation already_released;
      already_released.active = true;
      already_released.type = command.type;
      already_released.request_id = command.request_id;
      finishOperation(&already_released, true, kControlOk, "already_released",
                      state, responses, tracker);
      return;
    }
    if (state.motors_enabled) {
      // Manual charging can leave the uARCS motor-enabled flag set. Own the
      // safe transition: disable, confirm stable-off, then release charging.
      // This operation never sets velocity, enables motors, or clears E-stop.
      robot->lock();
      robot->disableMotors();
      robot->unlock();
      pending->phase = RELEASE_WAIT_MOTORS_DISABLED;
    } else {
      robot->lock();
      robot->comInt(kUarcsChargeCommand, 0);
      robot->unlock();
      pending->phase = RELEASE_WAIT_CHARGE_STATE;
    }
  } else if (command.type == patrolbot::COMMAND_MOTOR_ENABLE) {
    // Permitted only where the robot cannot still be connected to the dock.
    // There is no post-release window and no charge-state exception: enabling
    // motors in order to leave the dock is the UNDOCK operation's job.
    if (!patrolbot::motorEnableAllowed(observer->state(), state.estop_pressed)) {
      const char* reason =
          state.estop_pressed ? "estop_asserted" : "dock_contact_possible";
      rejectControl(command, kPreconditionFailed, reason, state, responses,
                    tracker);
      return;
    }
    responses->push_back(
        patrolbot::formatAck(command.type, command.request_id, false));
    if (state.motors_enabled) {
      PendingOperation already_enabled;
      already_enabled.active = true;
      already_enabled.type = command.type;
      already_enabled.request_id = command.request_id;
      finishOperation(&already_enabled, true, kControlOk, "already_enabled",
                      state, responses, tracker);
      return;
    }
    robot->lock();
    // Deliberate command only. DRIVE and startup never call enableMotors(),
    // and this path never touches the charger.
    robot->enableMotors();
    robot->unlock();
    pending->phase = MOTOR_ENABLE_WAIT_ENABLED;
  } else {
    rejectControl(command, kInvalidRequest, "unsupported_control", state,
                  responses, tracker);
    return;
  }

  pending->active = true;
  pending->type = command.type;
  pending->request_id = command.request_id;
  pending->stable_cycles = 0;
  pending->started.setToNow();
  tracker->begin(command.type, command.request_id);
  ArLog::log(ArLog::Normal, "%s %s accepted; awaiting hardware confirmation.",
             patrolbot::commandName(command.type).c_str(),
             command.request_id.c_str());
}

void updateControl(ArRobot* robot, PendingOperation* pending,
                   std::deque<std::string>* responses,
                   patrolbot::RequestTracker* tracker,
                   const patrolbot::DockObserver& observer,
                   const patrolbot::UndockHardwareProfile& profile) {
  if (!pending->active) return;
  HardwareSnapshot state = readSnapshot(robot);
  bool confirmed = false;
  if (pending->type == patrolbot::COMMAND_CHARGE_RELEASE) {
    if (!isStationary(state)) {
      robot->lock();
      robot->disableMotors();
      robot->unlock();
      state = readSnapshot(robot);
      finishOperation(pending, false, kHardwareRejected,
                      "motion_detected_during_release", state, responses,
                      tracker);
      return;
    }
    if (pending->phase == RELEASE_WAIT_MOTORS_DISABLED) {
      pending->stable_cycles =
          state.motors_enabled ? 0 : pending->stable_cycles + 1;
      if (pending->stable_cycles >= kStableCyclesRequired) {
        robot->lock();
        robot->comInt(kUarcsChargeCommand, 0);
        robot->unlock();
        pending->phase = RELEASE_WAIT_CHARGE_STATE;
        pending->stable_cycles = 0;
        ArLog::log(ArLog::Normal,
                   "CHARGE_RELEASE %s motors confirmed disabled; "
                   "charge release sent.",
                   pending->request_id.c_str());
      }
      if (pending->started.mSecSince() <= kOperationTimeoutMs) return;
    } else if (pending->phase == RELEASE_WAIT_CHARGE_STATE) {
      if (state.motors_enabled) {
        robot->lock();
        robot->disableMotors();
        robot->unlock();
        state = readSnapshot(robot);
        finishOperation(pending, false, kHardwareRejected,
                        "motors_enabled_during_release", state, responses,
                        tracker);
        return;
      }
      confirmed = state.charge_state == 0;
    }
  } else if (pending->type == patrolbot::COMMAND_MOTOR_ENABLE) {
    if (!patrolbot::motorEnableAllowed(observer.state(), state.estop_pressed) ||
        (state.fault_flags & profile.precheck_blocking_fault_mask) != 0 ||
        !isStationary(state)) {
      robot->lock();
      robot->setVel(0);
      robot->setRotVel(0);
      robot->disableMotors();
      robot->unlock();
      state = readSnapshot(robot);
      finishOperation(pending, false, kHardwareRejected,
                      "enable_precondition_changed", state, responses,
                      tracker);
      return;
    }
    confirmed = state.motors_enabled;
  }

  pending->stable_cycles = confirmed ? pending->stable_cycles + 1 : 0;
  if (pending->stable_cycles >= kStableCyclesRequired) {
    finishOperation(
        pending, true, kControlOk,
        pending->type == patrolbot::COMMAND_CHARGE_RELEASE
            ? "charge_released" : "motors_enabled",
        state, responses, tracker);
    return;
  }
  if (pending->started.mSecSince() > kOperationTimeoutMs) {
    if (pending->type == patrolbot::COMMAND_CHARGE_RELEASE &&
        state.motors_enabled) {
      robot->lock();
      robot->disableMotors();
      robot->unlock();
      state = readSnapshot(robot);
    }
    finishOperation(pending, false, kOperationTimeout, "hardware_timeout",
                    state, responses, tracker);
  }
}

std::string undockResultFor(const std::string& request_id, bool success,
                            int code, const std::string& message,
                            const patrolbot::UndockOutputs& out,
                            const patrolbot::UndockInputs& inputs,
                            const patrolbot::DockObserver& observer) {
  return patrolbot::formatUndockResult(
      request_id, success, code, message, out.phase, out.reverse_progress_m,
      out.lateral_error_m, out.heading_error_rad, inputs.charge_state,
      inputs.motors_enabled, inputs.estop_pressed, out.minimum_rear_range_m,
      out.release_attempts, observer.state(),
      observer.state() == patrolbot::DOCK_CLEAR_CONFIRMED);
}

// Records a terminal result for a request that never became a running
// operation, so the completed-result cache answers a replay identically.
void request_tracker_begin_and_finish(patrolbot::RequestTracker* tracker,
                                      const std::string& request_id,
                                      const std::string& result) {
  tracker->begin(patrolbot::COMMAND_UNDOCK, request_id);
  tracker->finish(patrolbot::COMMAND_UNDOCK, request_id, result);
}

void handleUndockRequest(const patrolbot::ParsedCommand& command,
                         patrolbot::UndockController* undock,
                         patrolbot::DockObserver* observer,
                         const patrolbot::UndockInputs& inputs,
                         const patrolbot::UndockHardwareProfile& profile,
                         std::deque<std::string>* responses,
                         patrolbot::RequestTracker* tracker,
                         UndockLeaseWatchdog* watchdog) {
  std::string completed_result;
  const patrolbot::RequestDisposition disposition = tracker->inspect(
      patrolbot::COMMAND_UNDOCK, command.request_id, &completed_result);
  if (disposition == patrolbot::REQUEST_COMPLETED) {
    // A replayed request id never runs a second manoeuvre.
    responses->push_back(patrolbot::formatAck(
        patrolbot::COMMAND_UNDOCK, command.request_id, true));
    responses->push_back(completed_result);
    return;
  }
  if (disposition == patrolbot::REQUEST_DUPLICATE_PENDING) {
    responses->push_back(patrolbot::formatAck(
        patrolbot::COMMAND_UNDOCK, command.request_id, true));
    return;
  }

  patrolbot::UndockOutputs rejected;
  if (disposition == patrolbot::REQUEST_BUSY) {
    responses->push_back(patrolbot::formatAck(
        patrolbot::COMMAND_UNDOCK, command.request_id, false));
    responses->push_back(undockResultFor(
        command.request_id, false, patrolbot::UNDOCK_BUSY,
        "another_operation_pending", rejected, inputs, *observer));
    return;
  }

  patrolbot::UndockConfig config;
  config.clearance_m = command.clearance_m;
  config.speed_mps = command.speed_mps;
  config.timeout_s = command.timeout_s;
  config.validation_mode = command.validation_mode;

  int code = 0;
  std::string message;
  responses->push_back(patrolbot::formatAck(
      patrolbot::COMMAND_UNDOCK, command.request_id, false));
  if (!undock->request(command.request_id, config, inputs, profile, observer,
                       &code, &message)) {
    const std::string result = undockResultFor(
        command.request_id, false, code, message, rejected, inputs, *observer);
    // Cache the rejection like every other terminal result, so replaying the
    // same request id returns the same answer instead of quietly re-evaluating
    // against different hardware state.
    request_tracker_begin_and_finish(tracker, command.request_id, result);
    responses->push_back(result);
    ArLog::log(ArLog::Terse, "UNDOCK %s refused: code=%d %s",
               command.request_id.c_str(), code, message.c_str());
    return;
  }
  tracker->begin(patrolbot::COMMAND_UNDOCK, command.request_id);
  watchdog->arm();
  responses->push_back(patrolbot::formatUndockEvent(
      command.request_id, undock->phase(), 0.0, 0.0, 0.0, inputs.charge_state,
      inputs.motors_enabled, inputs.rear.minimum_obstacle_range_m, 0,
      observer->state()));
  ArLog::log(ArLog::Normal,
             "UNDOCK %s accepted: clearance=%.3f m speed=%.3f m/s "
             "timeout=%.1f s validation=%d",
             command.request_id.c_str(), config.clearance_m, config.speed_mps,
             config.timeout_s, config.validation_mode ? 1 : 0);
}

void handleUndockCancel(const patrolbot::ParsedCommand& command,
                        patrolbot::UndockController* undock,
                        const patrolbot::UndockInputs& inputs,
                        const patrolbot::DockObserver& observer,
                        std::deque<std::string>* responses,
                        patrolbot::RequestTracker* tracker) {
  // Cancellation names the TARGET operation, so it can never go through
  // RequestTracker::inspect(): that keys on the command name and would call a
  // cancel of a pending UNDOCK "busy".
  responses->push_back(patrolbot::formatAck(
      patrolbot::COMMAND_UNDOCK_CANCEL, command.target_request_id, false));
  if (undock->cancel(command.target_request_id)) {
    ArLog::log(ArLog::Normal,
               "UNDOCK_CANCEL accepted for %s; stopping before reporting.",
               command.target_request_id.c_str());
    // Deliberately no terminal frame yet. The operation reports CANCELED only
    // once it has confirmed zero velocity and disabled the motors.
    return;
  }
  std::string completed_result;
  if (tracker->lookupCompleted(patrolbot::COMMAND_UNDOCK,
                               command.target_request_id, &completed_result)) {
    responses->push_back(completed_result);
    return;
  }
  patrolbot::UndockOutputs empty;
  responses->push_back(undockResultFor(
      command.target_request_id, false, patrolbot::UNDOCK_INVALID_REQUEST,
      "unknown_request", empty, inputs, observer));
}

}  // namespace

int main(int argc, char** argv) {
  Aria::init();
  ArArgumentParser parser(&argc, argv);
  parser.loadDefaultArguments();

  ArRobot robot;
  ArRobotConnector robotConnector(&parser, &robot);
  if (!robotConnector.parseArgs() || !parser.checkHelpAndWarnUnparsed()) {
    Aria::exit(1);
  }
  if (!robotConnector.connectRobot()) {
    ArLog::log(ArLog::Terse,
               "Error: Could not connect to local robot base.");
    Aria::exit(1);
  }

  robot.runAsync(true);
  robot.lock();
  // Sonar is telemetry and a safety input. Motors deliberately remain in their
  // hardware state; nothing here enables them.
  robot.enableSonar();
  robot.unlock();
  ArLog::log(ArLog::Normal,
             "Sonar enabled. Motors were NOT enabled at startup.");

  // ARIA already asks the base for its uARCS CONFIG packet during connection
  // negotiation and parses it, but nothing has ever read the result. It carries
  // the odometry calibration constants and the gyro configuration, none of which
  // are recorded anywhere. Log them once; this reads memory ARIA already holds
  // and issues no new command to the base.
  {
    const ArRobotConfigPacketReader* config = robot.getOrigRobotConfig();
    int waitedMs = 0;
    bool arrived = false;
    while (config != NULL && waitedMs < 3000) {
      robot.lock();
      arrived = config->hasPacketArrived();
      robot.unlock();
      if (arrived) break;
      ArUtil::sleep(100);
      waitedMs += 100;
    }
    if (arrived) {
      robot.lock();
      ArLog::log(ArLog::Normal,
                 "uARCS CONFIG: TicksMM=%d RevCount=%d DriftFactor=%d "
                 "HasGyro=%d GyroType=%d GyroFW=%d GyroCW=%d GyroCCW=%d",
                 config->getTicksMM(), config->getRevCount(),
                 config->getDriftFactor(), config->getHasGyro() ? 1 : 0,
                 config->getGyroType(), config->getGyroFWVersion(),
                 config->getGyroCW(), config->getGyroCCW());
      robot.unlock();
    } else {
      ArLog::log(ArLog::Terse,
                 "uARCS CONFIG: packet did not arrive within 3000 ms; "
                 "calibration constants unknown.");
    }
  }

  const patrolbot::UndockHardwareProfile profile = loadProfile(kProfilePath);

  ArLaserConnector laserConnector(&parser, &robot, &robotConnector);
  if (!laserConnector.connectLasers()) {
    ArLog::log(ArLog::Terse,
               "Warning: Target laser configuration not found.");
  }

  ArSocket serverSock;
  if (!serverSock.open(7272, ArSocket::TCP)) {
    ArLog::log(ArLog::Terse,
               "Error: TCP Port 7272 is already bound or restricted.");
    Aria::exit(1);
  }
  ArLog::log(ArLog::Normal,
             "PatrolBot text server listening on port 7272.");

  ArLaser* laser = robot.findLaser(1);
  if (!laser) laser = robot.findLaser(0);
  if (laser) {
    ArLog::log(ArLog::Normal, "Laser hooked: %s", laser->getName());
  } else {
    ArLog::log(ArLog::Terse,
               "WARNING: No active laser object found.");
  }

  PendingOperation pending;
  patrolbot::RequestTracker request_tracker;
  MonotonicClock clock;
  // Both survive client reconnects, so a replayed request id is answered
  // consistently and the dock state is not forgotten when the Pi restarts.
  // The observer starts UNKNOWN: nothing is assumed at process start.
  patrolbot::DockObserver dock_observer;
  patrolbot::UndockController undock;
  unsigned long applied_effect_seq = 0;

  UndockLeaseWatchdog watchdog(&robot, profile.lease_max_age_ms);
  watchdog.runAsync();

  while (robot.isRunning()) {
    ArLog::log(ArLog::Normal,
               "Waiting for the single Pi client on port 7272...");
    ArSocket clientSock;
    if (!serverSock.accept(&clientSock)) {
      ArLog::log(ArLog::Terse, "accept() failed, retrying in 1s...");
      ArUtil::sleep(1000);
      continue;
    }

    clientSock.setNonBlock();
    const int sockFd = clientSock.getFD();
    int one = 1;
    setsockopt(sockFd, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));
    int kaIdle = 2;
    int kaIntvl = 2;
    int kaCnt = 2;
    setsockopt(sockFd, IPPROTO_TCP, TCP_KEEPIDLE, &kaIdle, sizeof(kaIdle));
    setsockopt(sockFd, IPPROTO_TCP, TCP_KEEPINTVL, &kaIntvl, sizeof(kaIntvl));
    setsockopt(sockFd, IPPROTO_TCP, TCP_KEEPCNT, &kaCnt, sizeof(kaCnt));
    unsigned int userTimeoutMs = 5000;
    setsockopt(sockFd, IPPROTO_TCP, TCP_USER_TIMEOUT, &userTimeoutMs,
               sizeof(userTimeoutMs));

    ArLog::log(ArLog::Normal, "Pi connected. Streaming telemetry.");
    patrolbot::LineBuffer command_buffer;
    std::deque<std::string> responses;
    std::string pending_write;
    int eagainCount = 0;
    const int maxEagain = 300;
    ArTime lastCmdTime;
    lastCmdTime.setToNow();
    bool watchdogStopped = false;
    int auxCounter = 0;
    const int auxInterval = 5;
    bool client_connected = true;
    // A bridge that never sees FEATURES=UNDOCK refuses undock goals rather
    // than waiting for a reply that will not come.
    responses.push_back(patrolbot::formatHello());

    while (robot.isRunning() && client_connected) {
      const long long now_ms = clock.tick();
      const patrolbot::UndockInputs undock_inputs =
          readUndockInputs(&robot, profile, now_ms);
      dock_observer.update(undock_inputs, profile);

      // Drain all complete command lines. Fragmented and coalesced TCP reads are
      // handled by LineBuffer; no command inherits bytes from another command.
      char input[1024];
      const ssize_t bytesRead = ::recv(sockFd, input, sizeof(input), 0);
      if (bytesRead == 0) {
        client_connected = false;
      } else if (bytesRead > 0) {
        std::vector<std::string> lines;
        if (!command_buffer.append(input, bytesRead, &lines)) {
          ArLog::log(ArLog::Terse,
                     "Oversized command buffer discarded; stopping base.");
          if (!undock.active()) {
            robot.lock();
            robot.setVel(0);
            robot.setRotVel(0);
            robot.unlock();
          }
        }
        for (std::vector<std::string>::const_iterator it = lines.begin();
             it != lines.end(); ++it) {
          const patrolbot::ParsedCommand command = patrolbot::parseCommand(*it);
          if (command.type == patrolbot::COMMAND_DRIVE) {
            if (undock.active()) {
              // The exclusive undock owns motion. Do not touch velocity here at
              // all, not even to zero it: that would fight the controller.
              ArLog::log(ArLog::Terse,
                         "DRIVE refused: the UNDOCK state machine owns motion.");
              continue;
            }
            const HardwareSnapshot state = readSnapshot(&robot);
            const patrolbot::DriveDisposition drive =
                patrolbot::classifyDrive(
                    command.linear, command.angular, pending.active,
                    dock_observer.state(), state.motors_enabled,
                    state.estop_pressed, state.fault_flags,
                    profile.critical_fault_mask, false);
            const bool permitted = drive != patrolbot::DRIVE_REJECTED;
            const bool zero = drive == patrolbot::DRIVE_ALLOWED_ZERO;
            robot.lock();
            if (permitted) {
              robot.setVel(command.linear * 1000.0);
              robot.setRotVel(command.angular * 180.0 / M_PI);
            } else {
              robot.setVel(0);
              robot.setRotVel(0);
            }
            robot.unlock();
            lastCmdTime.setToNow();
            watchdogStopped = !permitted || zero;
            if (!permitted) {
              ArLog::log(ArLog::Terse,
                         "DRIVE refused: pending=%d dock_state=%s motors=%d "
                         "estop=%d fault=0x%x",
                         pending.active ? 1 : 0,
                         patrolbot::dockStateName(dock_observer.state()),
                         state.motors_enabled ? 1 : 0,
                         state.estop_pressed ? 1 : 0, state.fault_flags);
            }
          } else if (command.type == patrolbot::COMMAND_UNDOCK) {
            handleUndockRequest(command, &undock, &dock_observer, undock_inputs,
                                profile, &responses, &request_tracker,
                                &watchdog);
          } else if (command.type == patrolbot::COMMAND_UNDOCK_CANCEL) {
            handleUndockCancel(command, &undock, undock_inputs, dock_observer,
                               &responses, &request_tracker);
          } else if (command.type == patrolbot::COMMAND_CHARGE_RELEASE ||
                     command.type == patrolbot::COMMAND_MOTOR_ENABLE ||
                     command.type == patrolbot::COMMAND_DOCK_RESET) {
            startControl(command, &robot, &pending, &responses,
                         &request_tracker, &dock_observer, undock_inputs,
                         profile, undock.active());
          } else {
            ArLog::log(ArLog::Terse, "Rejected command: %s",
                       command.error.c_str());
          }
        }
      } else if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        client_connected = false;
      }

      updateControl(&robot, &pending, &responses, &request_tracker,
                    dock_observer, profile);

      // The independent watchdog thread has already stopped the base and
      // disabled the motors; invalidate the operation and report it.
      if (watchdog.tripped()) {
        watchdog.clearTrip();
        if (undock.active()) {
          const std::string request_id = undock.requestId();
          patrolbot::UndockOutputs terminal;
          terminal.phase = undock.phase();
          terminal.release_attempts = undock.releaseAttempts();
          undock.invalidate("command_lease_expired", &dock_observer);
          const std::string result = undockResultFor(
              request_id, false, patrolbot::UNDOCK_SAFETY_FAULT,
              "command_lease_expired", terminal, undock_inputs, dock_observer);
          request_tracker.finish(patrolbot::COMMAND_UNDOCK, request_id, result);
          responses.push_back(result);
        }
        watchdog.disarm();
      }

      MotionAuthority authority =
          undock.active() ? AUTHORITY_UNDOCK : AUTHORITY_DRIVE;
      if (undock.active()) {
        const patrolbot::UndockOutputs out = undock.step(undock_inputs);
        applyUndockEffects(&robot, out, &applied_effect_seq);
        robot.lock();
        robot.setVel(out.commanded_linear * 1000.0);
        // The SBC never rotates. The collision-aware turn away from the dock
        // stays in Nav2 and runs after this operation hands authority back.
        robot.setRotVel(0.0);
        robot.unlock();
        watchdog.renew();
        if (out.finished) {
          const std::string request_id = undock.requestId();
          const std::string result = undockResultFor(
              request_id, out.success, out.code, out.message, out, undock_inputs,
              dock_observer);
          request_tracker.finish(patrolbot::COMMAND_UNDOCK, request_id, result);
          responses.push_back(result);
          watchdog.disarm();
          authority = AUTHORITY_NONE;
          lastCmdTime.setToNow();
          // Success hands off with the motors LEFT ENABLED at zero velocity, so
          // Nav2 can take control; failure and cancellation have already
          // disabled them inside STOPPING.
          watchdogStopped = true;
          ArLog::log(out.success ? ArLog::Normal : ArLog::Terse,
                     "UNDOCK %s finished: success=%d code=%d %s "
                     "distance=%.3f attempts=%d dock_state=%s motors=%d",
                     request_id.c_str(), out.success ? 1 : 0, out.code,
                     out.message.c_str(), out.reverse_progress_m,
                     out.release_attempts,
                     patrolbot::dockStateName(dock_observer.state()),
                     undock_inputs.motors_enabled ? 1 : 0);
        } else if (out.phase_changed) {
          responses.push_back(patrolbot::formatUndockEvent(
              undock.requestId(), out.phase, out.reverse_progress_m,
              out.lateral_error_m, out.heading_error_rad,
              undock_inputs.charge_state, undock_inputs.motors_enabled,
              out.minimum_rear_range_m, out.release_attempts,
              dock_observer.state()));
        }
      }

      // The DRIVE watchdog applies only while DRIVE owns motion. Suppressing it
      // during an undock is safe only because the independent lease watchdog
      // above supervises the loop from outside.
      if (authority != AUTHORITY_UNDOCK && !watchdogStopped &&
          lastCmdTime.mSecSince() > 750) {
        robot.lock();
        robot.setVel(0);
        robot.setRotVel(0);
        robot.unlock();
        watchdogStopped = true;
      }

      HardwareSnapshot motion = readSnapshot(&robot);
      std::stringstream telemetry;
      robot.lock();
      const double x = robot.getX() / 1000.0;
      const double y = robot.getY() / 1000.0;
      const double th = robot.getTh() * M_PI / 180.0;
      robot.unlock();
      telemetry << "ODOM:" << x << "," << y << "," << th << ","
                << motion.linear << "," << motion.angular << "|LASER:";

      if (laser) {
        laser->lockDevice();
        const std::list<ArSensorReading*>* readings = laser->getRawReadings();
        bool laserFresh = false;
        if (laser->isConnected() && readings && !readings->empty()) {
          long newestAgeMs = 1000000;
          for (std::list<ArSensorReading*>::const_iterator it = readings->begin();
               it != readings->end(); ++it) {
            if (*it) {
              const long ageMs = (*it)->getTimeTaken().mSecSince();
              if (ageMs < newestAgeMs) newestAgeMs = ageMs;
            }
          }
          laserFresh = newestAgeMs <= 500;
        }
        if (laserFresh) {
          for (std::list<ArSensorReading*>::const_iterator it = readings->begin();
               it != readings->end(); ++it) {
            if (*it) telemetry << ((*it)->getRange() / 1000.0) << ",";
          }
        }
        laser->unlockDevice();
      }
      telemetry << "\n";

      if (++auxCounter >= auxInterval) {
        auxCounter = 0;
        robot.lock();
        // Emit SONAR every AUX cycle, including an empty field for a fresh clear scan.
        telemetry << "AUX:SONAR=";
        bool firstSonar = true;
        const int numSonar = robot.getNumSonar();
        for (int i = 0; i < numSonar; ++i) {
          ArSensorReading* reading = robot.getSonarReading(i);
          if (!reading) continue;
          const double range = reading->getRange();
          if (range >= 4335.0) continue;
          const double local_x =
              reading->getSensorX() + range * reading->getSensorDX();
          const double local_y =
              reading->getSensorY() + range * reading->getSensorDY();
          if (!firstSonar) telemetry << ";";
          telemetry << (local_x / 1000.0) << "," << (local_y / 1000.0);
          firstSonar = false;
        }
        const double soc = robot.haveStateOfCharge()
                               ? robot.getStateOfCharge() : -1.0;
        telemetry << "|BATT=" << robot.getRealBatteryVoltageNow() << ","
                  << soc << "," << robot.getChargeState() << ","
                  << static_cast<int>(robot.getTemperature());
        telemetry << "|FLAGS=" << robot.getFlags() << ","
                  << robot.getFaultFlags() << "," << robot.getStallValue()
                  << "," << (robot.areMotorsEnabled() ? 1 : 0) << ","
                  << (robot.isEStopPressed() ? 1 : 0);
        robot.unlock();
        // Dock ownership and inhibition state. Parsers skip unknown sections,
        // so adding this is backward compatible.
        telemetry << "|DOCK="
                  << patrolbot::dockStateName(dock_observer.state()) << ","
                  << static_cast<int>(undock.phase()) << ","
                  << patrolbot::undockPhaseName(undock.phase()) << ","
                  << (undock.active() ? 1 : 0) << ","
                  << undock.releaseAttempts() << ","
                  << undock_inputs.rear.minimum_obstacle_range_m << ","
                  << (undock_inputs.rear.usable() ? 1 : 0) << ","
                  << (dock_observer.redockInhibited(undock_inputs, profile) ? 1 : 0)
                  << ","
                  << dock_observer.redockInhibitRemainingMs(undock_inputs,
                                                            profile)
                  << "," << (profile.commissioned() ? 1 : 0) << "\n";
      }

      // Control replies and the telemetry frame are coalesced into one buffer
      // per iteration, so an undock event no longer costs a whole telemetry
      // frame. Partial writes are retained; telemetry is latest-only.
      if (pending_write.empty()) {
        while (!responses.empty() &&
               pending_write.size() < kMaxPendingWrite) {
          pending_write += responses.front();
          responses.pop_front();
        }
        if (pending_write.size() + telemetry.str().size() <= kMaxPendingWrite) {
          pending_write += telemetry.str();
        }
      }
      const ssize_t bytesWritten = ::send(
          sockFd, pending_write.data(), pending_write.size(), MSG_NOSIGNAL);
      if (bytesWritten > 0) {
        pending_write.erase(0, static_cast<std::size_t>(bytesWritten));
        eagainCount = 0;
      } else if (bytesWritten < 0 &&
                 (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
        if (++eagainCount >= maxEagain) client_connected = false;
      } else {
        client_connected = false;
      }
      ArUtil::sleep(50);
    }

    // Loss of the client stops the base, disables the motors and invalidates
    // any operation in flight. Nothing resumes on reconnect.
    robot.lock();
    robot.setVel(0);
    robot.setRotVel(0);
    robot.stop();
    // Losing the Pi means nothing is steering the robot any more, so the motors
    // come down unconditionally. This is stricter than the previous behaviour,
    // which only disabled them when an operation happened to be in flight, and
    // it means a reconnecting bridge must issue an explicit MOTOR_ENABLE before
    // anything moves again.
    robot.disableMotors();
    robot.unlock();
    watchdog.disarm();
    watchdog.clearTrip();
    if (undock.active()) {
      const std::string request_id = undock.requestId();
      patrolbot::UndockOutputs terminal;
      terminal.phase = undock.phase();
      terminal.release_attempts = undock.releaseAttempts();
      undock.invalidate("client_session_changed", &dock_observer);
      const patrolbot::UndockInputs final_inputs =
          readUndockInputs(&robot, profile, clock.tick());
      request_tracker.finish(
          patrolbot::COMMAND_UNDOCK, request_id,
          undockResultFor(request_id, false, patrolbot::UNDOCK_SESSION_CHANGED,
                          "client_session_changed", terminal, final_inputs,
                          dock_observer));
    }
    if (pending.active) {
      const HardwareSnapshot state = readSnapshot(&robot);
      finishOperation(&pending, false, kSessionChanged,
                      "client_session_changed", state, &responses,
                      &request_tracker, false);
    }
    responses.clear();
    clientSock.close();
    ArLog::log(ArLog::Normal,
               "Pi disconnected; base stopped and awaiting reconnect.");
  }

  watchdog.requestStop();
  serverSock.close();
  Aria::exit(0);
}
