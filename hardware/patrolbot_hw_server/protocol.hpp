#ifndef PATROLBOT_PROTOCOL_HPP
#define PATROLBOT_PROTOCOL_HPP

#include <cstddef>
#include <deque>
#include <map>
#include <string>
#include <vector>

namespace patrolbot {

enum CommandType {
  COMMAND_INVALID = 0,
  COMMAND_DRIVE,
  COMMAND_CHARGE_RELEASE,
  COMMAND_MOTOR_ENABLE
};

enum RequestDisposition {
  REQUEST_NEW = 0,
  REQUEST_DUPLICATE_PENDING,
  REQUEST_BUSY,
  REQUEST_COMPLETED
};

enum DriveDisposition {
  DRIVE_REJECTED = 0,
  DRIVE_ALLOWED_ZERO,
  DRIVE_ALLOWED_NORMAL,
  DRIVE_ALLOWED_DOCK_DEPARTURE,
  DRIVE_ALLOWED_DOCK_TURN
};

struct ParsedCommand {
  ParsedCommand();

  CommandType type;
  std::string request_id;
  double linear;
  double angular;
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
DriveDisposition classifyDrive(
    double linear, double angular, bool control_pending, int charge_state,
    bool motors_enabled, bool estop_pressed, unsigned int fault_flags,
    bool dock_departure_authorized, bool dock_turn_authorized);
bool motorEnableChargeAllowed(int charge_state, bool recent_release);
std::string commandName(CommandType command);
std::string formatAck(CommandType command, const std::string& request_id,
                      bool replayed);
std::string formatResult(CommandType command, const std::string& request_id,
                         bool success, int code, const std::string& message,
                         int charge_state, bool motors_enabled,
                         bool estop_pressed);

}  // namespace patrolbot

#endif  // PATROLBOT_PROTOCOL_HPP
