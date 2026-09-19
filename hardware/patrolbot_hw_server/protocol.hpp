#ifndef PATROLBOT_PROTOCOL_HPP
#define PATROLBOT_PROTOCOL_HPP

#include <cstddef>
#include <deque>
#include <map>
#include <string>
#include <vector>

// DockPhysicalState and UndockPhase are part of the wire surface, so the
// protocol layer needs them. undock.hpp defines the enums plus header-only name
// helpers, which keeps this a compile-time dependency only: protocol.cpp still
// links without undock.cpp.
#include "undock.hpp"

namespace patrolbot {

// Wire protocol version advertised in the HELLO banner. A Pi bridge that does
// not see FEATURES=UNDOCK refuses undock goals rather than hanging.
const int kProtocolVersion = 2;

enum CommandType {
  COMMAND_INVALID = 0,
  COMMAND_DRIVE,
  COMMAND_CHARGE_RELEASE,
  COMMAND_MOTOR_ENABLE,
  COMMAND_UNDOCK,
  COMMAND_UNDOCK_CANCEL,
  COMMAND_DOCK_RESET
};

enum RequestDisposition {
  REQUEST_NEW = 0,
  REQUEST_DUPLICATE_PENDING,
  REQUEST_BUSY,
  REQUEST_COMPLETED
};

// There is no dock-departure or dock-turn disposition. Motion during dock
// contact belongs exclusively to the UNDOCK state machine, which commands the
// base directly and never goes through this classifier.
enum DriveDisposition {
  DRIVE_REJECTED = 0,
  DRIVE_ALLOWED_ZERO,
  DRIVE_ALLOWED_NORMAL
};

struct ParsedCommand {
  ParsedCommand();

  CommandType type;
  std::string request_id;
  // UNDOCK_CANCEL carries the id of the operation being cancelled, not a new
  // request id of its own.
  std::string target_request_id;
  double linear;
  double angular;
  double clearance_m;
  double speed_mps;
  double timeout_s;
  bool validation_mode;
  std::string error;
};

class LineBuffer {
 public:
  explicit LineBuffer(std::size_t maximum_size = 4096);
  bool append(const char* data, std::size_t size, std::vector<std::string>* lines);
  void clear();

 private:
  std::size_t maximum_size_;
  std::string buffer_;
};

class RequestTracker {
 public:
  explicit RequestTracker(std::size_t completed_limit = 64);
  RequestDisposition inspect(CommandType command, const std::string& request_id,
                             std::string* completed_result) const;
  void begin(CommandType command, const std::string& request_id);
  void finish(CommandType command, const std::string& request_id,
              const std::string& result);
  bool hasPending() const;
  // UNDOCK_CANCEL deliberately bypasses inspect(): it names a different command
  // than the operation it targets, so the ordinary busy/duplicate rules would
  // reject it. These let the dispatcher find and complete the tracked UNDOCK.
  bool pendingMatches(CommandType command, const std::string& request_id) const;
  bool lookupCompleted(CommandType command, const std::string& request_id,
                       std::string* completed_result) const;

 private:
  std::string key(CommandType command, const std::string& request_id) const;
  std::size_t completed_limit_;
  bool pending_;
  CommandType pending_command_;
  std::string pending_request_id_;
  std::map<std::string, std::string> completed_;
  std::deque<std::string> completed_order_;
};

bool isCanonicalUuid(const std::string& value);
ParsedCommand parseCommand(const std::string& line);

// Ordinary motion. There is no time input and no charge_state input: permission
// comes from the persistent dock observer having proven the robot clear, never
// from a window being open.
DriveDisposition classifyDrive(double linear, double angular,
                               bool control_pending,
                               DockPhysicalState dock_state,
                               bool motors_enabled, bool estop_pressed,
                               unsigned int fault_flags,
                               unsigned int critical_fault_mask,
                               bool undock_active);

// MOTOR_ENABLE is a decoupled maintenance command: it never releases charging
// and is refused whenever the robot may still be connected to the dock.
bool motorEnableAllowed(DockPhysicalState dock_state, bool estop_pressed);

std::string commandName(CommandType command);
std::string formatAck(CommandType command, const std::string& request_id,
                      bool replayed);
std::string formatResult(CommandType command, const std::string& request_id,
                         bool success, int code, const std::string& message,
                         int charge_state, bool motors_enabled,
                         bool estop_pressed);

// Progress and terminal frames for UNDOCK. Distinct leading verbs, so a bridge
// that predates them ignores the lines instead of raising a parse error.
std::string formatUndockEvent(const std::string& request_id, UndockPhase phase,
                              double distance_m, double lateral_m,
                              double heading_rad, int charge_state,
                              bool motors_enabled, double rear_range_m,
                              int release_attempts,
                              DockPhysicalState dock_state);
std::string formatUndockResult(const std::string& request_id, bool success,
                               int code, const std::string& message,
                               UndockPhase phase, double distance_m,
                               double lateral_m, double heading_rad,
                               int charge_state, bool motors_enabled,
                               bool estop_pressed, double rear_range_m,
                               int release_attempts,
                               DockPhysicalState dock_state,
                               bool physically_clear);
std::string formatHello(const std::string& odom_epoch = "");

}  // namespace patrolbot

#endif  // PATROLBOT_PROTOCOL_HPP
