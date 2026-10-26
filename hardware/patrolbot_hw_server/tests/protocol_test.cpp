// Offline tests for the SBC/Pi wire protocol and the ordinary drive gate.
//
// Failures report file:line rather than aborting on the first bad assertion, so
// one run reports every regression.

#include "../protocol.hpp"

#include <cmath>
#include <cstdio>
#include <string>
#include <stdexcept>
#include <vector>

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

#define RUN(name)                                                            \
  g_case = #name;                                                            \
  name();

using namespace patrolbot;

const std::string kId = "123e4567-e89b-12d3-a456-426614174000";
const std::string kOtherId = "aaaaaaaa-bbbb-4ccc-8ddd-eeeeeeeeeeee";
// bit 2 | bit 8 | bit 9 | bit 15, matching UndockHardwareProfile's default.
const unsigned int kCritical = 0x8304;

void test_uuid_validation() {
  CHECK(isCanonicalUuid(kId));
  CHECK(!isCanonicalUuid("123e4567-e89b-12d3-a456-42661417400Z"));
  CHECK(!isCanonicalUuid("123E4567-E89B-12D3-A456-426614174000"));
  CHECK(!isCanonicalUuid(""));
  CHECK(!isCanonicalUuid("123e4567e89b12d3a456426614174000"));
}

void test_maintenance_command_parsing() {
  ParsedCommand release = parseCommand("CHARGE_RELEASE:" + kId);
  CHECK(release.type == COMMAND_CHARGE_RELEASE);
  CHECK(release.request_id == kId);

  ParsedCommand enable = parseCommand("MOTOR_ENABLE:" + kId);
  CHECK(enable.type == COMMAND_MOTOR_ENABLE);

  ParsedCommand disable = parseCommand("MOTOR_DISABLE:" + kId);
  CHECK(disable.type == COMMAND_MOTOR_DISABLE);
  CHECK(commandName(disable.type) == "MOTOR_DISABLE");
  CHECK(parseCommand("MOTOR_DISABLE:not-a-uuid").type == COMMAND_INVALID);

  ParsedCommand reset = parseCommand("DOCK_RESET:" + kId);
  CHECK(reset.type == COMMAND_DOCK_RESET);
  CHECK(reset.request_id == kId);

  CHECK(parseCommand("MOTOR_ENABLE:not-a-uuid").type == COMMAND_INVALID);
  CHECK(parseCommand("DOCK_RESET:not-a-uuid").type == COMMAND_INVALID);
}

void test_lidar_power_protocol() {
  CHECK(parseCommand("LIDAR_ON:" + kId).type == COMMAND_LIDAR_ON);
  CHECK(parseCommand("LIDAR_OFF:" + kId).type == COMMAND_LIDAR_OFF);
  CHECK(parseCommand("LIDAR_OFF:" + kId).request_id == kId);
  CHECK(parseCommand("LIDAR_ON:not-a-uuid").type == COMMAND_INVALID);
  CHECK(parseCommand("LIDAR_ON:" + kId + ":").type == COMMAND_INVALID);
  CHECK(parseCommand("LIDAR_OFF:" + kId + ":extra").type == COMMAND_INVALID);
  CHECK(formatHello().find("FEATURES=UNDOCK,LIDAR_POWER") != std::string::npos);
  CHECK(formatResult(COMMAND_LIDAR_OFF, kId, true, 0, "lidar_off_commanded",
                     0, false, false) ==
        "RESULT:LIDAR_OFF:" + kId + ":OK:0:lidar_off_commanded:0:0:0\n");

  RequestTracker tracker;
  std::string cached;
  tracker.begin(COMMAND_LIDAR_OFF, kId);
  CHECK(tracker.inspect(COMMAND_LIDAR_OFF, kId, &cached) == REQUEST_DUPLICATE_PENDING);
  CHECK(tracker.inspect(COMMAND_LIDAR_ON, kOtherId, &cached) == REQUEST_BUSY);
  tracker.finish(COMMAND_LIDAR_OFF, kId, "off_result");
  CHECK(tracker.inspect(COMMAND_LIDAR_OFF, kId, &cached) == REQUEST_COMPLETED);
  CHECK(cached == "off_result");
  CHECK(tracker.inspect(COMMAND_LIDAR_ON, kId, &cached) == REQUEST_NEW);
}

void test_drive_command_parsing() {
  ParsedCommand drive = parseCommand("DRIVE:-0.05:0.0");
  CHECK(drive.type == COMMAND_DRIVE);
  CHECK(std::fabs(drive.linear + 0.05) < 1e-9);
  CHECK(parseCommand("DRIVE:nan:0").type == COMMAND_INVALID);
  CHECK(parseCommand("DRIVE:inf:0").type == COMMAND_INVALID);
  CHECK(parseCommand("DRIVE:0.1abc:0").type == COMMAND_INVALID);
  CHECK(parseCommand("DRIVE:0.1").type == COMMAND_INVALID);
}

void test_undock_command_parsing() {
  ParsedCommand undock = parseCommand("UNDOCK:" + kId + ":0.53:0.10:30:0");
  CHECK(undock.type == COMMAND_UNDOCK);
  CHECK(undock.request_id == kId);
  CHECK(std::fabs(undock.clearance_m - 0.53) < 1e-9);
  CHECK(std::fabs(undock.speed_mps - 0.10) < 1e-9);
  CHECK(std::fabs(undock.timeout_s - 30.0) < 1e-9);
  CHECK(!undock.validation_mode);

  ParsedCommand validation = parseCommand("UNDOCK:" + kId + ":0.10:0.05:20:1");
  CHECK(validation.type == COMMAND_UNDOCK);
  CHECK(validation.validation_mode);

  // Every field is mandatory and strictly typed.
  CHECK(parseCommand("UNDOCK:" + kId + ":0.53:0.10:30").type ==
        COMMAND_INVALID);
  CHECK(parseCommand("UNDOCK:" + kId + ":0.53:0.10:30:2").type ==
        COMMAND_INVALID);
  CHECK(parseCommand("UNDOCK:" + kId + ":nan:0.10:30:0").type ==
        COMMAND_INVALID);
  CHECK(parseCommand("UNDOCK:not-a-uuid:0.53:0.10:30:0").type ==
        COMMAND_INVALID);
}

void test_undock_cancel_names_its_target() {
  ParsedCommand cancel = parseCommand("UNDOCK_CANCEL:" + kId);
  CHECK(cancel.type == COMMAND_UNDOCK_CANCEL);
  // The payload is the id of the operation being cancelled, not a new one.
  CHECK(cancel.target_request_id == kId);
  CHECK(parseCommand("UNDOCK_CANCEL:not-a-uuid").type == COMMAND_INVALID);
}

void test_drive_requires_a_proven_clear_dock_state() {
  // Zero is always accepted so a stop can never be refused.
  CHECK(classifyDrive(0.0, 0.0, true, DOCK_DOCKED_CONFIRMED, false, true,
                      0xffff, kCritical, true, true) == DRIVE_ALLOWED_ZERO);

  CHECK(classifyDrive(-0.05, 0.0, false, DOCK_CLEAR_CONFIRMED, true, false, 0,
                      kCritical, false, true) == DRIVE_ALLOWED_NORMAL);
  CHECK(classifyDrive(0.0, 0.70, false, DOCK_CLEAR_CONFIRMED, true, false, 0,
                      kCritical, false, true) == DRIVE_ALLOWED_NORMAL);

  // Every state other than CLEAR_CONFIRMED refuses motion, including UNKNOWN
  // at process start.
  CHECK(classifyDrive(-0.05, 0.0, false, DOCK_UNKNOWN, true, false, 0,
                      kCritical, false, true) == DRIVE_REJECTED);
  CHECK(classifyDrive(-0.05, 0.0, false, DOCK_DOCKED_CONFIRMED, true, false, 0,
                      kCritical, false, true) == DRIVE_REJECTED);
  CHECK(classifyDrive(-0.05, 0.0, false, DOCK_DEPARTING, true, false, 0,
                      kCritical, false, true) == DRIVE_REJECTED);
  CHECK(classifyDrive(-0.05, 0.0, false, DOCK_FAULT_STOPPED, true, false, 0,
                      kCritical, false, true) == DRIVE_REJECTED);
}

void test_drive_is_refused_while_undock_owns_motion() {
  // The exact reverse the undock itself commands is still refused when it
  // arrives as an ordinary DRIVE, in every dock state.
  CHECK(classifyDrive(-0.10, 0.0, false, DOCK_DEPARTING, true, false, 0,
                      kCritical, true, true) == DRIVE_REJECTED);
  CHECK(classifyDrive(-0.10, 0.0, false, DOCK_CLEAR_CONFIRMED, true, false, 0,
                      kCritical, true, true) == DRIVE_REJECTED);
  CHECK(classifyDrive(0.0, 0.5, false, DOCK_CLEAR_CONFIRMED, true, false, 0,
                      kCritical, true, true) == DRIVE_REJECTED);
  // A stop is still honoured during an undock.
  CHECK(classifyDrive(0.0, 0.0, false, DOCK_DEPARTING, true, false, 0,
                      kCritical, true, true) == DRIVE_ALLOWED_ZERO);
}

void test_drive_bounds_and_hardware_gates() {
  CHECK(classifyDrive(0.31, 0.0, false, DOCK_CLEAR_CONFIRMED, true, false, 0,
                      kCritical, false, true) == DRIVE_REJECTED);
  CHECK(classifyDrive(0.0, 1.21, false, DOCK_CLEAR_CONFIRMED, true, false, 0,
                      kCritical, false, true) == DRIVE_REJECTED);
  CHECK(classifyDrive(-0.05, 0.0, true, DOCK_CLEAR_CONFIRMED, true, false, 0,
                      kCritical, false, true) == DRIVE_REJECTED);
  CHECK(classifyDrive(-0.05, 0.0, false, DOCK_CLEAR_CONFIRMED, false, false, 0,
                      kCritical, false, true) == DRIVE_REJECTED);
  CHECK(classifyDrive(-0.05, 0.0, false, DOCK_CLEAR_CONFIRMED, true, true, 0,
                      kCritical, false, true) == DRIVE_REJECTED);

  // Critical faults block; a non-critical flag such as low_battery (bit 3)
  // does not, because a robot that has just left the dock reports it normally.
  CHECK(classifyDrive(-0.05, 0.0, false, DOCK_CLEAR_CONFIRMED, true, false,
                      0x0100, kCritical, false, true) == DRIVE_REJECTED);
  CHECK(classifyDrive(-0.05, 0.0, false, DOCK_CLEAR_CONFIRMED, true, false,
                      0x0008, kCritical, false, true) == DRIVE_ALLOWED_NORMAL);
}

void test_lidar_gates_motion_but_never_stop() {
  CHECK(classifyDrive(.05, 0, false, DOCK_CLEAR_CONFIRMED, true, false, 0,
                      kCritical, false, false) == DRIVE_REJECTED);
  CHECK(classifyDrive(0, 0, true, DOCK_UNKNOWN, false, true, 0xffff,
                      kCritical, false, false) == DRIVE_ALLOWED_ZERO);
  CHECK(motorEnableAllowed(DOCK_CLEAR_CONFIRMED, false, false));
  CHECK(motorEnableAllowed(DOCK_UNKNOWN, false, false));
}

void test_motor_enable_is_decoupled_from_charging() {
  // Motor power does not release charging or authorize movement.
  CHECK(motorEnableAllowed(DOCK_CLEAR_CONFIRMED, false, true));
  CHECK(motorEnableAllowed(DOCK_UNKNOWN, false, true));
  CHECK(motorEnableAllowed(DOCK_DOCKED_CONFIRMED, false, true));
  CHECK(motorEnableAllowed(DOCK_DEPARTING, false, true));
  CHECK(motorEnableAllowed(DOCK_FAULT_STOPPED, false, true));
  // E-stop always refuses, and there is no "recent release" back door.
  CHECK(!motorEnableAllowed(DOCK_CLEAR_CONFIRMED, true, true));
  CHECK(!motorEnableAllowed(DOCK_UNKNOWN, true, true));
}

void test_line_buffer_reassembles_fragments() {
  LineBuffer buffer;
  std::vector<std::string> lines;
  CHECK(buffer.append("CHARGE_RE", 9, &lines));
  CHECK(lines.empty());
  const std::string remainder = "LEASE:" + kId + "\nDRIVE:0:0\r\n";
  CHECK(buffer.append(remainder.data(), remainder.size(), &lines));
  CHECK(lines.size() == 2);
  CHECK(lines[0] == "CHARGE_RELEASE:" + kId);
  CHECK(lines[1] == "DRIVE:0:0");

  LineBuffer small(16);
  std::vector<std::string> overflow;
  CHECK(!small.append("0123456789012345678901234567890", 31, &overflow));
}

void test_control_frame_formatting() {
  UndockHardwareProfile bumpers;
  bumpers.bumper_mapping_valid = true;
  bumpers.front_bumper_mask = 0x7e00;
  bumpers.rear_bumper_mask = 0x007e;
  CHECK(bumperBlocksDrive(.05, 0., 0x0200, bumpers));
  CHECK(!bumperBlocksDrive(-.05, 0., 0x0200, bumpers));
  CHECK(bumperBlocksDrive(-.05, 0., 0x0002, bumpers));
  CHECK(!bumperBlocksDrive(.05, 0., 0x0002, bumpers));
  CHECK(bumperBlocksDrive(0., .1, 0x0200, bumpers));
  CHECK(bumperBlocksDrive(0., -.1, 0x0002, bumpers));
  CHECK(!bumperBlocksDrive(0., 0., 0x7e7e, bumpers));
  CHECK(formatAck(COMMAND_MOTOR_ENABLE, kId, true) ==
        "ACK:MOTOR_ENABLE:" + kId + ":REPLAYED\n");
  CHECK(formatAck(COMMAND_UNDOCK, kId, false) ==
        "ACK:UNDOCK:" + kId + ":ACCEPTED\n");
  CHECK(formatResult(COMMAND_CHARGE_RELEASE, kId, true, 0, "charge released", 0,
                     false, false) ==
        "RESULT:CHARGE_RELEASE:" + kId + ":OK:0:charge_released:0:0:0\n");
  CHECK(formatHello() == "HELLO:PROTOCOL=2:FEATURES=UNDOCK,LIDAR_POWER,MOTOR_DISABLE,MANUAL_UNDOCK_V1\n");
  CHECK(formatHello(kId) ==
        "HELLO:PROTOCOL=2:FEATURES=UNDOCK,LIDAR_POWER,MOTOR_DISABLE,MANUAL_UNDOCK_V1:ODOM_EPOCH=" + kId + "\n");
  const ParsedCommand manual = parseCommand("MANUAL_UNDOCK:" + kId + ":.53:.1:45:0");
  CHECK(manual.type == COMMAND_UNDOCK && manual.manual_mode && !manual.validation_mode);
  CHECK(parseCommand("UNDOCK:" + kId + ":.53:.1:45:0").manual_mode == false);
  CHECK(parseCommand("MANUAL_UNDOCK:" + kId + ":.53:.1:45:1").type == COMMAND_INVALID);
  bool rejected = false;
  try { formatHello("bad:ODOM_EPOCH=injection"); }
  catch (const std::invalid_argument&) { rejected = true; }
  CHECK(rejected);
}

void test_undock_frame_formatting() {
  // Distinct leading verbs, so a bridge that predates them ignores the lines
  // instead of raising a parse error on an unexpected RESULT arity.
  const std::string event =
      formatUndockEvent(kId, UNDOCK_PHASE_BACKING_OUT, 0.1234, 0.0056, 0.0102,
                        3, true, 1.5, 2, DOCK_DEPARTING);
  CHECK(event == "UNDOCK_EVENT:" + kId +
                     ":4:BACKING_OUT:0.1234:0.0056:0.0102:3:1:1.5000:2:"
                     "DEPARTING\n");

  const std::string result = formatUndockResult(
      kId, true, 0, "undocked", UNDOCK_PHASE_HANDOFF_READY, 0.5300, 0.0010,
      0.0020, 3, true, false, 99.0, 1, DOCK_CLEAR_CONFIRMED, true);
  CHECK(result == "UNDOCK_RESULT:" + kId +
                      ":OK:0:undocked:6:HANDOFF_READY:0.5300:0.0010:0.0020:3:1:"
                      "0:99.0000:1:CLEAR_CONFIRMED:1\n");

  // Messages are sanitised to a single protocol-safe token.
  const std::string failed = formatUndockResult(
      kId, false, UNDOCK_SENSOR_BLOCKED, "rear path blocked",
      UNDOCK_PHASE_STOPPING, 0.12, 0.0, 0.0, 0, false, false, 0.1, 3,
      DOCK_FAULT_STOPPED, false);
  CHECK(failed.find(":rear_path_blocked:") != std::string::npos);
  CHECK(failed.find(":FAULT_STOPPED:0\n") != std::string::npos);
}

void test_request_tracker_lifecycle() {
  RequestTracker tracker(2);
  std::string cached;
  CHECK(tracker.inspect(COMMAND_UNDOCK, kId, &cached) == REQUEST_NEW);
  tracker.begin(COMMAND_UNDOCK, kId);
  CHECK(tracker.inspect(COMMAND_UNDOCK, kId, &cached) ==
        REQUEST_DUPLICATE_PENDING);
  // A different command while an undock is pending is busy, which is exactly
  // why cancellation cannot go through inspect().
  CHECK(tracker.inspect(COMMAND_CHARGE_RELEASE, kId, &cached) == REQUEST_BUSY);
  CHECK(tracker.inspect(COMMAND_UNDOCK, kOtherId, &cached) == REQUEST_BUSY);
  CHECK(tracker.pendingMatches(COMMAND_UNDOCK, kId));
  CHECK(!tracker.pendingMatches(COMMAND_UNDOCK, kOtherId));

  const std::string terminal = formatUndockResult(
      kId, true, 0, "undocked", UNDOCK_PHASE_HANDOFF_READY, 0.53, 0.0, 0.0, 0,
      true, false, 99.0, 0, DOCK_CLEAR_CONFIRMED, true);
  tracker.finish(COMMAND_UNDOCK, kId, terminal);
  CHECK(!tracker.hasPending());
  CHECK(tracker.inspect(COMMAND_UNDOCK, kId, &cached) == REQUEST_COMPLETED);
  CHECK(cached == terminal);
  CHECK(tracker.lookupCompleted(COMMAND_UNDOCK, kId, &cached));
  CHECK(!tracker.lookupCompleted(COMMAND_UNDOCK, kOtherId, &cached));
}

}  // namespace

int main() {
  RUN(test_uuid_validation);
  RUN(test_maintenance_command_parsing);
  RUN(test_lidar_power_protocol);
  RUN(test_drive_command_parsing);
  RUN(test_undock_command_parsing);
  RUN(test_undock_cancel_names_its_target);
  RUN(test_drive_requires_a_proven_clear_dock_state);
  RUN(test_drive_is_refused_while_undock_owns_motion);
  RUN(test_drive_bounds_and_hardware_gates);
  RUN(test_lidar_gates_motion_but_never_stop);
  RUN(test_motor_enable_is_decoupled_from_charging);
  RUN(test_line_buffer_reassembles_fragments);
  RUN(test_control_frame_formatting);
  RUN(test_undock_frame_formatting);
  RUN(test_request_tracker_lifecycle);

  if (g_failures != 0) {
    std::fprintf(stderr, "\nprotocol_test: %d of %d checks FAILED\n",
                 g_failures, g_checks);
    return 1;
  }
  std::fprintf(stderr, "protocol_test: %d checks passed\n", g_checks);
  return 0;
}
