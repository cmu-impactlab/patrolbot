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

}  // namespace

ParsedCommand::ParsedCommand()
    : type(COMMAND_INVALID), linear(0.0), angular(0.0) {}

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
      (fields[0] == "CHARGE_RELEASE" || fields[0] == "MOTOR_ENABLE")) {
    if (!isCanonicalUuid(fields[1])) {
      command.error = "request ID is not a canonical UUID";
      return command;
    }
    command.type = fields[0] == "CHARGE_RELEASE"
                       ? COMMAND_CHARGE_RELEASE : COMMAND_MOTOR_ENABLE;
    command.request_id = fields[1];
    return command;
  }
  command.error = "unknown or malformed command";
  return command;
}

DriveDisposition classifyDrive(
    double linear, double angular, bool control_pending, int charge_state,
    bool motors_enabled, bool estop_pressed, unsigned int fault_flags,
    bool dock_departure_authorized, bool dock_turn_authorized) {
  const bool zero =
      std::fabs(linear) < 1e-9 && std::fabs(angular) < 1e-9;
  if (zero) return DRIVE_ALLOWED_ZERO;

  const bool bounded =
      std::fabs(linear) <= 0.30 && std::fabs(angular) <= 1.20;
  if (!bounded || control_pending || !motors_enabled || estop_pressed ||
      fault_flags != 0) {
    return DRIVE_REJECTED;
  }
  if (charge_state == 0) return DRIVE_ALLOWED_NORMAL;

  // A recently released, explicitly motor-enabled robot may see charging
  // reassert while its contacts are still physically on the dock. Admit only
  // the straight, bounded reverse needed to clear those contacts. The caller
  // owns the short authorization timer and revokes it on any unsafe transition.
  if (dock_departure_authorized &&
      linear < -1e-9 && linear >= -0.100001 &&
      std::fabs(angular) < 1e-9) {
    return DRIVE_ALLOWED_DOCK_DEPARTURE;
  }

  // After the robot has cleared all commissioned dock contacts, the manager
  // performs a second explicit release-and-enable pair before its 180-degree
  // Nav2 Spin. Some µARCS bases reassert charge state 3 even away from the
  // contacts. Admit only a bounded in-place turn during that separately scoped
  // authorization; translation and arcs remain rejected.
  if (dock_turn_authorized &&
      std::fabs(linear) < 1e-9 &&
      std::fabs(angular) > 1e-9 &&
      std::fabs(angular) <= 0.700001) {
    return DRIVE_ALLOWED_DOCK_TURN;
  }
  return DRIVE_REJECTED;
}

bool motorEnableChargeAllowed(int charge_state, bool recent_release) {
  if (charge_state == 0) return true;
  // The PatrolBot can reassert charge state 3 immediately after a confirmed
  // CHARGE 0 even without a front bumper bit. A same-session recent release is
  // the authorization; the server reissues CHARGE 0 atomically with enable.
  return charge_state > 0 && recent_release;
}

std::string commandName(CommandType command) {
  switch (command) {
    case COMMAND_DRIVE: return "DRIVE";
    case COMMAND_CHARGE_RELEASE: return "CHARGE_RELEASE";
    case COMMAND_MOTOR_ENABLE: return "MOTOR_ENABLE";
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

}  // namespace patrolbot
