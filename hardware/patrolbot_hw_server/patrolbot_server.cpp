#include <new>
#include <iterator>
#include <cstddef>
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
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "Aria.h"
#include "protocol.hpp"

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
// PatrolBot third-generation user guide: µARCS CHARGE is command 68,
// argument 0 releases the autocharge mechanism. This legacy ARIA install does
// not define ArCommands::CHARGE (its generic enum gives 68 a simulator alias),
// so use the manual's hardware-protocol number explicitly.
const unsigned char kUarcsChargeCommand = 68;

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

HardwareSnapshot readSnapshot(ArRobot* robot) {
  HardwareSnapshot state;
  robot->lock();
  state.linear = robot->getVel() / 1000.0;
  state.angular = robot->getRotVel() * M_PI / 180.0;
  state.charge_state = robot->getChargeState();
  state.motors_enabled = robot->areMotorsEnabled();
  state.estop_pressed = robot->isEStopPressed();
  state.fault_flags = robot->getFaultFlags();
  robot->unlock();
  return state;
}

bool isStationary(const HardwareSnapshot& state) {
  return std::fabs(state.linear) <= 0.005 &&
         std::fabs(state.angular) <= 0.01;
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

void startControl(const patrolbot::ParsedCommand& command, ArRobot* robot,
                  PendingOperation* pending,
                  std::deque<std::string>* responses,
                  patrolbot::RequestTracker* tracker) {
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
  if (!isStationary(state)) {
    rejectControl(command, kPreconditionFailed, "robot_not_stationary", state,
                  responses, tracker);
    return;
  }
  if (state.fault_flags != 0) {
    rejectControl(command, kPreconditionFailed, "hardware_fault", state,
                  responses, tracker);
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
      // Manual charging can leave the µARCS motor-enabled flag set. Own the
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
    if (state.charge_state != 0) {
      rejectControl(command, kPreconditionFailed, "charging", state, responses,
                    tracker);
      return;
    }
    if (state.estop_pressed) {
      rejectControl(command, kPreconditionFailed, "estop_asserted", state,
                    responses, tracker);
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
    // Deliberate command only. DRIVE and startup never call enableMotors().
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
                   patrolbot::RequestTracker* tracker) {
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
    if (state.charge_state != 0 || state.estop_pressed) {
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
  // Sonar is telemetry-only. Motors deliberately remain in their hardware state.
  robot.enableSonar();
  robot.unlock();
  ArLog::log(ArLog::Normal,
             "Sonar enabled. Motors were NOT enabled at startup.");

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

    while (robot.isRunning() && client_connected) {
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
          robot.lock();
          robot.setVel(0);
          robot.setRotVel(0);
          robot.unlock();
        }
        for (std::vector<std::string>::const_iterator it = lines.begin();
             it != lines.end(); ++it) {
          const patrolbot::ParsedCommand command = patrolbot::parseCommand(*it);
          if (command.type == patrolbot::COMMAND_DRIVE) {
            const bool zero = std::fabs(command.linear) < 1e-9 &&
                              std::fabs(command.angular) < 1e-9;
            HardwareSnapshot state = readSnapshot(&robot);
            const bool bounded = std::fabs(command.linear) <= 0.30 &&
                                 std::fabs(command.angular) <= 1.20;
            const bool permitted = zero ||
                (bounded && !pending.active && state.charge_state == 0 &&
                 !state.estop_pressed && state.motors_enabled &&
                 state.fault_flags == 0);
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
                         "DRIVE refused: bounded=%d pending=%d charge=%d motors=%d estop=%d",
                         bounded ? 1 : 0, pending.active ? 1 : 0,
                         state.charge_state, state.motors_enabled ? 1 : 0,
                         state.estop_pressed ? 1 : 0);
            }
          } else if (command.type == patrolbot::COMMAND_CHARGE_RELEASE ||
                     command.type == patrolbot::COMMAND_MOTOR_ENABLE) {
            startControl(command, &robot, &pending, &responses,
                         &request_tracker);
          } else {
            ArLog::log(ArLog::Terse, "Rejected command: %s",
                       command.error.c_str());
          }
        }
      } else if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        client_connected = false;
      }

      updateControl(&robot, &pending, &responses, &request_tracker);

      if (!watchdogStopped && lastCmdTime.mSecSince() > 750) {
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
                  << (robot.isEStopPressed() ? 1 : 0) << "\n";
        robot.unlock();
      }

      // Control replies are sent before the next telemetry frame and retained
      // across partial writes. Telemetry is latest-only under back-pressure.
      if (pending_write.empty()) {
        if (!responses.empty()) {
          pending_write = responses.front();
          responses.pop_front();
        } else {
          pending_write = telemetry.str();
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

    robot.lock();
    robot.setVel(0);
    robot.setRotVel(0);
    robot.stop();
    if (pending.active) {
      robot.disableMotors();
    }
    robot.unlock();
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

  serverSock.close();
  Aria::exit(0);
}
