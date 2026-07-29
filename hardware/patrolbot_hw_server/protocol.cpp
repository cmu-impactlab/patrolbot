#include "protocol.hpp"

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <sstream>

namespace patrolbot {
namespace {

bool isHex(char value) {
  return (value >= '0' && value <= '9') ||
         (value >= 'a' && value <= 'f');
}

bool parseFiniteDouble(const std::string& text, double* result) {
  if (text.empty()) return false;
  char* end = NULL;
  errno = 0;
  const double value = std::strtod(text.c_str(), &end);
  if (errno != 0 || end == text.c_str() || *end != '\0' || !std::isfinite(value)) {
    return false;
  }
  *result = value;
  return true;
}

std::vector<std::string> split(const std::string& text, char delimiter) {
  std::vector<std::string> fields;
  std::stringstream stream(text);
  std::string field;
  while (std::getline(stream, field, delimiter)) fields.push_back(field);
  return fields;
}

std::string safeToken(const std::string& message) {
  std::string result;
  for (std::string::const_iterator it = message.begin(); it != message.end(); ++it) {
    const char value = *it;
    if ((value >= 'a' && value <= 'z') ||
        (value >= 'A' && value <= 'Z') ||
        (value >= '0' && value <= '9') || value == '-' || value == '_') {
      result.push_back(value);
    } else {
      result.push_back('_');
    }
  }
  return result.empty() ? "unspecified" : result;
}

// Fixed precision keeps the wire deterministic. The default ostream format
// switches to scientific notation for small magnitudes, which would produce
// fields the Pi parser has to guess at.
std::string fixed4(double value) {
  if (!std::isfinite(value)) return "0.0000";
  std::stringstream stream;
  stream.setf(std::ios::fixed, std::ios::floatfield);
  stream.precision(4);
  stream << value;
  return stream.str();
}

}  // namespace

ParsedCommand::ParsedCommand()
    : type(COMMAND_INVALID), linear(0.0), angular(0.0), clearance_m(0.0),
      speed_mps(0.0), timeout_s(0.0), validation_mode(false) {}

LineBuffer::LineBuffer(std::size_t maximum_size)
    : maximum_size_(maximum_size) {}

bool LineBuffer::append(const char* data, std::size_t size,
                        std::vector<std::string>* lines) {
  if (data == NULL || lines == NULL) return false;
  buffer_.append(data, size);
  if (buffer_.size() > maximum_size_) {
    buffer_.clear();
    return false;
  }
  std::size_t newline = std::string::npos;
  while ((newline = buffer_.find('\n')) != std::string::npos) {
    std::string line = buffer_.substr(0, newline);
    buffer_.erase(0, newline + 1);
    if (!line.empty() && line[line.size() - 1] == '\r') line.erase(line.size() - 1);
    if (!line.empty()) lines->push_back(line);
  }
  return true;
}

void LineBuffer::clear() { buffer_.clear(); }

RequestTracker::RequestTracker(std::size_t completed_limit)
    : completed_limit_(completed_limit), pending_(false),
      pending_command_(COMMAND_INVALID) {}

std::string RequestTracker::key(CommandType command,
                                const std::string& request_id) const {
  return commandName(command) + ":" + request_id;
}

RequestDisposition RequestTracker::inspect(
    CommandType command, const std::string& request_id,
    std::string* completed_result) const {
  const std::string request_key = key(command, request_id);
  std::map<std::string, std::string>::const_iterator completed =
      completed_.find(request_key);
  if (completed != completed_.end()) {
    if (completed_result != NULL) *completed_result = completed->second;
    return REQUEST_COMPLETED;
  }
  if (!pending_) return REQUEST_NEW;
  if (pending_command_ == command && pending_request_id_ == request_id) {
    return REQUEST_DUPLICATE_PENDING;
  }
  return REQUEST_BUSY;
}

void RequestTracker::begin(CommandType command, const std::string& request_id) {
  pending_ = true;
  pending_command_ = command;
  pending_request_id_ = request_id;
}

void RequestTracker::finish(CommandType command, const std::string& request_id,
                            const std::string& result) {
  const std::string request_key = key(command, request_id);
  completed_[request_key] = result;
  completed_order_.push_back(request_key);
  while (completed_order_.size() > completed_limit_) {
    completed_.erase(completed_order_.front());
    completed_order_.pop_front();
  }
  if (pending_ && pending_command_ == command &&
      pending_request_id_ == request_id) {
    pending_ = false;
    pending_command_ = COMMAND_INVALID;
    pending_request_id_.clear();
  }
}

bool RequestTracker::hasPending() const { return pending_; }

bool RequestTracker::pendingMatches(CommandType command,
                                    const std::string& request_id) const {
  return pending_ && pending_command_ == command &&
         pending_request_id_ == request_id;
}

bool RequestTracker::lookupCompleted(CommandType command,
                                     const std::string& request_id,
                                     std::string* completed_result) const {
  std::map<std::string, std::string>::const_iterator found =
      completed_.find(key(command, request_id));
  if (found == completed_.end()) return false;
  if (completed_result != NULL) *completed_result = found->second;
  return true;
}

bool isCanonicalUuid(const std::string& value) {
  if (value.size() != 36) return false;
  for (std::size_t i = 0; i < value.size(); ++i) {
    if (i == 8 || i == 13 || i == 18 || i == 23) {
      if (value[i] != '-') return false;
    } else if (!isHex(value[i])) {
      return false;
    }
  }
  return true;
}

ParsedCommand parseCommand(const std::string& line) {
  ParsedCommand command;
  const std::vector<std::string> fields = split(line, ':');
  if (fields.empty()) {
    command.error = "empty command";
    return command;
  }
  if (fields.size() == 3 && fields[0] == "DRIVE") {
    if (!parseFiniteDouble(fields[1], &command.linear) ||
        !parseFiniteDouble(fields[2], &command.angular)) {
      command.error = "malformed DRIVE velocity";
      return command;
    }
    command.type = COMMAND_DRIVE;
    return command;
  }
  if (fields.size() == 2 &&
      (fields[0] == "CHARGE_RELEASE" || fields[0] == "MOTOR_ENABLE" ||
       fields[0] == "DOCK_RESET")) {
    if (!isCanonicalUuid(fields[1])) {
      command.error = "request ID is not a canonical UUID";
      return command;
    }
    if (fields[0] == "CHARGE_RELEASE") {
      command.type = COMMAND_CHARGE_RELEASE;
    } else if (fields[0] == "MOTOR_ENABLE") {
      command.type = COMMAND_MOTOR_ENABLE;
    } else {
      command.type = COMMAND_DOCK_RESET;
    }
    command.request_id = fields[1];
    return command;
  }
  if (fields.size() == 2 && fields[0] == "UNDOCK_CANCEL") {
    if (!isCanonicalUuid(fields[1])) {
      command.error = "target request ID is not a canonical UUID";
      return command;
    }
    command.type = COMMAND_UNDOCK_CANCEL;
    // The payload names the operation being cancelled. There is no separate
    // cancel request id, and this never enters the ordinary request tracker.
    command.target_request_id = fields[1];
    command.request_id = fields[1];
    return command;
  }
  if (fields.size() == 6 && fields[0] == "UNDOCK") {
    if (!isCanonicalUuid(fields[1])) {
      command.error = "request ID is not a canonical UUID";
      return command;
    }
    if (!parseFiniteDouble(fields[2], &command.clearance_m) ||
        !parseFiniteDouble(fields[3], &command.speed_mps) ||
        !parseFiniteDouble(fields[4], &command.timeout_s)) {
      command.error = "malformed UNDOCK parameter";
      return command;
    }
    if (fields[5] != "0" && fields[5] != "1") {
      command.error = "malformed UNDOCK validation flag";
      return command;
    }
    command.validation_mode = fields[5] == "1";
    command.type = COMMAND_UNDOCK;
    command.request_id = fields[1];
    return command;
  }
  command.error = "unknown or malformed command";
  return command;
}

DriveDisposition classifyDrive(double linear, double angular,
                               bool control_pending,
                               DockPhysicalState dock_state,
                               bool motors_enabled, bool estop_pressed,
                               unsigned int fault_flags,
                               unsigned int critical_fault_mask,
                               bool undock_active) {
  const bool zero =
      std::fabs(linear) < 1e-9 && std::fabs(angular) < 1e-9;
  if (zero) return DRIVE_ALLOWED_ZERO;

  // While the exclusive undock state machine owns motion, nothing else moves
  // the robot. This is a state, not a window, so there is nothing to expire.
  if (undock_active) return DRIVE_REJECTED;

  const bool bounded =
      std::fabs(linear) <= 0.30 && std::fabs(angular) <= 1.20;
  if (!bounded || control_pending || !motors_enabled || estop_pressed ||
      (fault_flags & critical_fault_mask) != 0) {
    return DRIVE_REJECTED;
  }

  // charge_state is deliberately absent. A robot whose departure was proven by
  // measured displacement and clear sensors stays drivable even if the µARCS
  // controller re-asserts a stale charge level, and a robot that has not proven
  // that is refused however its charge state happens to read.
  if (dock_state != DOCK_CLEAR_CONFIRMED) return DRIVE_REJECTED;
  return DRIVE_ALLOWED_NORMAL;
}

bool motorEnableAllowed(DockPhysicalState dock_state, bool estop_pressed) {
  if (estop_pressed) return false;
  // Refused while the robot may still be physically connected to the dock.
  // Releasing the charger in order to move is the UNDOCK operation's job, and
  // this command never touches charging.
  return dock_state == DOCK_CLEAR_CONFIRMED || dock_state == DOCK_UNKNOWN;
}

std::string commandName(CommandType command) {
  switch (command) {
    case COMMAND_DRIVE: return "DRIVE";
    case COMMAND_CHARGE_RELEASE: return "CHARGE_RELEASE";
    case COMMAND_MOTOR_ENABLE: return "MOTOR_ENABLE";
    case COMMAND_UNDOCK: return "UNDOCK";
    case COMMAND_UNDOCK_CANCEL: return "UNDOCK_CANCEL";
    case COMMAND_DOCK_RESET: return "DOCK_RESET";
    default: return "INVALID";
  }
}

std::string formatAck(CommandType command, const std::string& request_id,
                      bool replayed) {
  std::stringstream stream;
  stream << "ACK:" << commandName(command) << ":" << request_id << ":"
         << (replayed ? "REPLAYED" : "ACCEPTED") << "\n";
  return stream.str();
}

std::string formatResult(CommandType command, const std::string& request_id,
                         bool success, int code, const std::string& message,
                         int charge_state, bool motors_enabled,
                         bool estop_pressed) {
  std::stringstream stream;
  stream << "RESULT:" << commandName(command) << ":" << request_id << ":"
         << (success ? "OK" : "ERROR") << ":" << code << ":"
         << safeToken(message) << ":" << charge_state << ":"
         << (motors_enabled ? 1 : 0) << ":" << (estop_pressed ? 1 : 0)
         << "\n";
  return stream.str();
}

std::string formatUndockEvent(const std::string& request_id, UndockPhase phase,
                              double distance_m, double lateral_m,
                              double heading_rad, int charge_state,
                              bool motors_enabled, double rear_range_m,
                              int release_attempts,
                              DockPhysicalState dock_state) {
  std::stringstream stream;
  stream << "UNDOCK_EVENT:" << request_id << ":" << static_cast<int>(phase)
         << ":" << undockPhaseName(phase) << ":" << fixed4(distance_m) << ":"
         << fixed4(lateral_m) << ":" << fixed4(heading_rad) << ":"
         << charge_state << ":" << (motors_enabled ? 1 : 0) << ":"
         << fixed4(rear_range_m) << ":" << release_attempts << ":"
         << dockStateName(dock_state) << "\n";
  return stream.str();
}

std::string formatUndockResult(const std::string& request_id, bool success,
                               int code, const std::string& message,
                               UndockPhase phase, double distance_m,
                               double lateral_m, double heading_rad,
                               int charge_state, bool motors_enabled,
                               bool estop_pressed, double rear_range_m,
                               int release_attempts,
                               DockPhysicalState dock_state,
                               bool physically_clear) {
  std::stringstream stream;
  stream << "UNDOCK_RESULT:" << request_id << ":"
         << (success ? "OK" : "ERROR") << ":" << code << ":"
         << safeToken(message) << ":" << static_cast<int>(phase) << ":"
         << undockPhaseName(phase) << ":" << fixed4(distance_m) << ":"
         << fixed4(lateral_m) << ":" << fixed4(heading_rad) << ":"
         << charge_state << ":" << (motors_enabled ? 1 : 0) << ":"
         << (estop_pressed ? 1 : 0) << ":" << fixed4(rear_range_m) << ":"
         << release_attempts << ":" << dockStateName(dock_state) << ":"
         << (physically_clear ? 1 : 0) << "\n";
  return stream.str();
}

std::string formatHello() {
  std::stringstream stream;
  stream << "HELLO:PROTOCOL=" << kProtocolVersion << ":FEATURES=UNDOCK\n";
  return stream.str();
}

}  // namespace patrolbot
