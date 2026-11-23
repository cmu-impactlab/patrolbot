#ifndef PATROLBOT_SIP_ODOMETRY_HPP
#define PATROLBOT_SIP_ODOMETRY_HPP

#include "Aria.h"

#include <ostream>

namespace patrolbot {

struct SipOdometryStamp {
  bool confirmed;
  ArTime time;

  void appendTo(std::ostream& wire, const ArTime& assembled_at) const {
    // Older ARIA headers implement the time-to-time mSecSinceLL overload
    // with the opposite sign. Subtract the same-clock fields explicitly.
    const auto sip_ms = time.getSecLL() * 1000 + time.getMSecLL();
    const auto assembly_ms = assembled_at.getSecLL() * 1000 + assembled_at.getMSecLL();
    wire << "|ODOM_SOURCE_MS:";
    if (!confirmed || !ArTime::usingMonotonicClock() || assembly_ms < sip_ms) {
      wire << "-1,-1";
      return;
    }
    wire << sip_ms << ',' << (assembly_ms - sip_ms);
  }
};

// getLastOdometryTime() is set by processMotorPacket(), but is also seeded
// during connection setup. Corroborate it with an actual SIP observed in the
// packet handler and read it under the same robot lock as pose and twist.
class SipOdometryObserver {
 public:
  explicit SipOdometryObserver(ArRobot& robot)
      : robot_(robot), callback_(this, &SipOdometryObserver::packet) {
    robot_.addPacketHandler(&callback_, ArListPos::FIRST);
  }
  ~SipOdometryObserver() { robot_.remPacketHandler(&callback_); }
  SipOdometryObserver(const SipOdometryObserver&) = delete;
  SipOdometryObserver& operator=(const SipOdometryObserver&) = delete;

  // The caller holds the robot lock. A valid packet received in the same
  // millisecond has an indistinguishable time; the bridge checks for conflicts.
  SipOdometryStamp stamp() const {
    const ArTime processed = robot_.getLastOdometryTime();
    return {seen_ && processed == expected_, processed};
  }

 private:
  bool packet(ArRobotPacket* packet) {
    if (packet->getID() == 0x32 || packet->getID() == 0x33) {
      expected_ = packet->getTimeReceived();
      // Match ARIA's adjustment when applying this packet. Do not modify
      // the packet/read cursor or take over ARIA's motor-packet processing.
      seen_ = expected_.addMSec(-static_cast<long>(robot_.getOdometryDelay()));
    }
    return false;
  }

  ArRobot& robot_;
  ArRetFunctor1C<bool, SipOdometryObserver, ArRobotPacket*> callback_;
  ArTime expected_;
  bool seen_ = false;
};

}  // namespace patrolbot

#endif
