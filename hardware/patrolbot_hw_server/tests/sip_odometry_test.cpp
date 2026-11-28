#include "../sip_odometry.hpp"

#include <cassert>
#include <sstream>

// Exercise the linked ARIA's real packet dispatch and odometry update without
// creating a device connection, starting a robot loop, or sending a command.
class OfflineRobot : public ArRobot {
 public:
  using ArRobot::handlePacket;
};

static ArTime at(unsigned long ms) {
  ArTime time;
  time.setSecLL(100);
  time.setMSecLL(ms);
  return time;
}

static std::string wire(OfflineRobot& robot, const patrolbot::SipOdometryObserver& source,
                        unsigned long assembled_ms) {
  robot.lock();
  const auto stamp = source.stamp();
  robot.unlock();
  std::ostringstream out;
  stamp.appendTo(out, at(assembled_ms));
  return out.str();
}

static void feed(OfflineRobot& robot, unsigned char id, unsigned long ms, short x = 0) {
  ArRobotPacket packet;
  packet.setID(id);
  if (id == 0x90) {
    packet.uByte4ToBuf(10);
    packet.uByte4ToBuf(20);
  } else {
    packet.uByte2ToBuf(x);
    // Y, heading, left/right wheel velocity, battery, stall, control,
    // flags, compass, and sonar count (zero), in standard SIP order.
    for (int i = 0; i < 17; ++i) packet.uByteToBuf(0);
  }
  packet.finalizePacket();
  packet.resetRead();
  packet.setTimeReceived(at(ms));
  assert(robot.handlePacket(&packet));
}

int main() {
  Aria::init();
  OfflineRobot robot;
  robot.setOdometryDelay(7);
  patrolbot::SipOdometryObserver source(robot);

  assert(wire(robot, source, 50) == "|ODOM_SOURCE_MS:-1,-1");
  feed(robot, 0x90, 60);
  assert(wire(robot, source, 65) == "|ODOM_SOURCE_MS:-1,-1");
  assert(ArTime::usingMonotonicClock());

  feed(robot, 0x32, 100);
  const auto first_pose = robot.getPose();
  assert(wire(robot, source, 120) == "|ODOM_SOURCE_MS:100093,27");
  feed(robot, 0x90, 140);
  assert(wire(robot, source, 150) == "|ODOM_SOURCE_MS:100093,57");
  assert(wire(robot, source, 90) == "|ODOM_SOURCE_MS:-1,-1");

  feed(robot, 0x33, 200);  // New stationary sample, identical pose and twist.
  assert(robot.getPose() == first_pose);
  assert(wire(robot, source, 220) == "|ODOM_SOURCE_MS:100193,27");
  feed(robot, 0x32, 300, 20);
  assert(robot.getX() != first_pose.getX());
  assert(wire(robot, source, 305) == "|ODOM_SOURCE_MS:100293,12");
}
