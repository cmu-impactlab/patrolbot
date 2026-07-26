#include "../protocol.hpp"

#include <cassert>
#include <cmath>
#include <string>
#include <vector>

int main() {
  const std::string id = "123e4567-e89b-12d3-a456-426614174000";
  assert(patrolbot::isCanonicalUuid(id));
  assert(!patrolbot::isCanonicalUuid("123e4567-e89b-12d3-a456-42661417400Z"));

  patrolbot::ParsedCommand release =
      patrolbot::parseCommand("CHARGE_RELEASE:" + id);
  assert(release.type == patrolbot::COMMAND_CHARGE_RELEASE);
  assert(release.request_id == id);
  assert(release.linear == 0.0 && release.angular == 0.0);

  patrolbot::ParsedCommand enable =
      patrolbot::parseCommand("MOTOR_ENABLE:" + id);
  assert(enable.type == patrolbot::COMMAND_MOTOR_ENABLE);
  assert(enable.linear == 0.0 && enable.angular == 0.0);

  patrolbot::ParsedCommand drive = patrolbot::parseCommand("DRIVE:-0.05:0.0");
  assert(drive.type == patrolbot::COMMAND_DRIVE);
  assert(std::fabs(drive.linear + 0.05) < 1e-9);
  assert(patrolbot::parseCommand("DRIVE:nan:0").type == patrolbot::COMMAND_INVALID);
  assert(patrolbot::parseCommand("MOTOR_ENABLE:not-a-uuid").type ==
         patrolbot::COMMAND_INVALID);

  assert(patrolbot::classifyDrive(
             0.0, 0.0, true, 3, false, true, 99, false, false) ==
         patrolbot::DRIVE_ALLOWED_ZERO);
  assert(patrolbot::classifyDrive(
             -0.05, 0.0, false, 0, true, false, 0, false, false) ==
         patrolbot::DRIVE_ALLOWED_NORMAL);
  assert(patrolbot::classifyDrive(
             -0.10, 0.0, false, 3, true, false, 0, true, false) ==
         patrolbot::DRIVE_ALLOWED_DOCK_DEPARTURE);
  assert(patrolbot::classifyDrive(
             -0.100002, 0.0, false, 3, true, false, 0, true, false) ==
         patrolbot::DRIVE_REJECTED);
  assert(patrolbot::classifyDrive(
             0.01, 0.0, false, 3, true, false, 0, true, false) ==
         patrolbot::DRIVE_REJECTED);
  assert(patrolbot::classifyDrive(
             -0.01, 0.01, false, 3, true, false, 0, true, false) ==
         patrolbot::DRIVE_REJECTED);
  assert(patrolbot::classifyDrive(
             -0.01, 0.0, true, 3, true, false, 0, true, false) ==
         patrolbot::DRIVE_REJECTED);
  assert(patrolbot::classifyDrive(
             -0.01, 0.0, false, 3, true, true, 0, true, false) ==
         patrolbot::DRIVE_REJECTED);
  assert(patrolbot::classifyDrive(
             -0.01, 0.0, false, 3, true, false, 1, true, false) ==
         patrolbot::DRIVE_REJECTED);
  assert(patrolbot::classifyDrive(
             -0.01, 0.0, false, 3, true, false, 0, false, false) ==
         patrolbot::DRIVE_REJECTED);
  assert(patrolbot::classifyDrive(
             0.0, 0.70, false, 3, true, false, 0, true, true) ==
         patrolbot::DRIVE_ALLOWED_DOCK_TURN);
  assert(patrolbot::classifyDrive(
             0.0, -0.70, false, 3, true, false, 0, true, true) ==
         patrolbot::DRIVE_ALLOWED_DOCK_TURN);
  assert(patrolbot::classifyDrive(
             0.0, 0.700002, false, 3, true, false, 0, true, true) ==
         patrolbot::DRIVE_REJECTED);
  assert(patrolbot::classifyDrive(
             0.001, 0.20, false, 3, true, false, 0, true, true) ==
         patrolbot::DRIVE_REJECTED);
  assert(patrolbot::classifyDrive(
             0.0, 0.20, false, 3, true, false, 0, true, false) ==
         patrolbot::DRIVE_REJECTED);

  assert(patrolbot::motorEnableChargeAllowed(0, false));
  assert(patrolbot::motorEnableChargeAllowed(3, true));
  assert(!patrolbot::motorEnableChargeAllowed(3, false));
  assert(!patrolbot::motorEnableChargeAllowed(-1, true));

  patrolbot::LineBuffer buffer;
  std::vector<std::string> lines;
  assert(buffer.append("CHARGE_RE", 9, &lines));
  assert(lines.empty());
  const std::string remainder = "LEASE:" + id + "\nDRIVE:0:0\n";
  assert(buffer.append(remainder.data(), remainder.size(), &lines));
  assert(lines.size() == 2);
  assert(lines[0] == "CHARGE_RELEASE:" + id);
  assert(lines[1] == "DRIVE:0:0");

  assert(patrolbot::formatAck(patrolbot::COMMAND_MOTOR_ENABLE, id, true) ==
         "ACK:MOTOR_ENABLE:" + id + ":REPLAYED\n");
  assert(patrolbot::formatResult(
             patrolbot::COMMAND_CHARGE_RELEASE, id, true, 0,
             "charge released", 0, false, false) ==
         "RESULT:CHARGE_RELEASE:" + id +
             ":OK:0:charge_released:0:0:0\n");

  patrolbot::RequestTracker tracker(2);
  std::string cached;
  assert(tracker.inspect(patrolbot::COMMAND_CHARGE_RELEASE, id, &cached) ==
         patrolbot::REQUEST_NEW);
  tracker.begin(patrolbot::COMMAND_CHARGE_RELEASE, id);
  assert(tracker.inspect(patrolbot::COMMAND_CHARGE_RELEASE, id, &cached) ==
         patrolbot::REQUEST_DUPLICATE_PENDING);
  assert(tracker.inspect(patrolbot::COMMAND_MOTOR_ENABLE, id, &cached) ==
         patrolbot::REQUEST_BUSY);
  const std::string terminal = patrolbot::formatResult(
      patrolbot::COMMAND_CHARGE_RELEASE, id, true, 0, "released", 0, false,
      false);
  tracker.finish(patrolbot::COMMAND_CHARGE_RELEASE, id, terminal);
  assert(!tracker.hasPending());
  assert(tracker.inspect(patrolbot::COMMAND_CHARGE_RELEASE, id, &cached) ==
         patrolbot::REQUEST_COMPLETED);
  assert(cached == terminal);
  return 0;
}
