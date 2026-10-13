#ifndef PATROLBOT_LIDAR_POWER_HPP
#define PATROLBOT_LIDAR_POWER_HPP

#include "scan_timing.hpp"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <fcntl.h>
#include <fstream>
#include <string>
#include <unistd.h>

namespace patrolbot {

enum LidarState { LIDAR_UNKNOWN, LIDAR_OFF, LIDAR_STARTING, LIDAR_READY, LIDAR_FAULT };

inline const char* lidarStateName(LidarState state) {
  switch (state) {
    case LIDAR_OFF: return "OFF";
    case LIDAR_STARTING: return "STARTING";
    case LIDAR_READY: return "READY";
    case LIDAR_FAULT: return "FAULT";
    default: return "UNKNOWN";
  }
}

// OFF means the power command was sent, never electrical readback.
class LidarPower {
 public:
  explicit LidarPower(bool commissioned, LidarState restored = LIDAR_UNKNOWN)
      : supported_(commissioned), state_(restored), on_at_ms_(-1),
        command_failed_(restored == LIDAR_FAULT) {}
  bool supported() const { return supported_; }
  bool ready() const { return state_ == LIDAR_READY; }
  LidarState state() const { return state_; }
  bool requestAllowed(double linear, double angular, bool base_fresh,
                      bool operation_pending) const {
    return supported_ && base_fresh && !operation_pending &&
        state_ != LIDAR_STARTING && std::isfinite(linear) &&
        std::isfinite(angular) && std::fabs(linear) <= 0.005 &&
        std::fabs(angular) <= 0.01;
  }
  bool shouldReconnect() const {
    return state_ != LIDAR_OFF && !command_failed_;
  }
  void onRequested(long long now_ms) {
    state_ = LIDAR_STARTING;
    on_at_ms_ = now_ms;
    command_failed_ = false;
  }
  void offRequested() { state_ = LIDAR_OFF; command_failed_ = false; }
  void commandFailed() { state_ = LIDAR_FAULT; command_failed_ = true; }
  void observe(long age_ms, long long now_ms) {
    if (state_ == LIDAR_OFF || command_failed_) return;
    const bool fresh = age_ms >= 0 && age_ms <= kMaxScanAgeMs &&
        (on_at_ms_ < 0 || age_ms < now_ms - on_at_ms_);
    if (fresh) state_ = LIDAR_READY;
    // This deadline reports failed startup; only measured scans establish READY.
    else if (state_ == LIDAR_STARTING && now_ms - on_at_ms_ <= 60000) return;
    else if (state_ != LIDAR_UNKNOWN) state_ = LIDAR_FAULT;
  }

 private:
  bool supported_;
  LidarState state_;
  long long on_at_ms_;
  bool command_failed_;
};

// Scope the remembered operator state to an SBC boot. Routine service/TCP
// restarts must not undo OFF; actual boot-on belongs to commissioned firmware.
inline LidarState loadLidarState(const std::string& path,
                                const std::string& boot_id) {
  if (boot_id.empty()) return LIDAR_FAULT;
  errno = 0;
  std::ifstream file(path.c_str());
  if (!file.is_open()) return errno == ENOENT ? LIDAR_UNKNOWN : LIDAR_FAULT;
  std::string saved_boot, state, extra;
  if (!(file >> saved_boot >> state) || (file >> extra)) return LIDAR_FAULT;
  if (saved_boot != boot_id) return LIDAR_UNKNOWN;
  if (state == "OFF") return LIDAR_OFF;
  if (state == "ON") return LIDAR_UNKNOWN;
  return LIDAR_FAULT;
}

inline bool saveLidarState(const std::string& path, const std::string& boot_id,
                          LidarState state) {
  if (boot_id.empty()) return false;
  const std::string temporary = path + ".tmp";
  const std::string content = boot_id + " " +
      (state == LIDAR_OFF ? "OFF" : state == LIDAR_STARTING ? "ON" : "FAULT") + "\n";
  const int fd = open(temporary.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_NOFOLLOW, 0600);
  if (fd < 0) return false;
  const bool written = write(fd, content.data(), content.size()) ==
                       static_cast<ssize_t>(content.size());
  const bool synced = fsync(fd) == 0;
  const bool closed = close(fd) == 0;
  if (!written || !synced || !closed || rename(temporary.c_str(), path.c_str()) != 0) {
    std::remove(temporary.c_str());
    return false;
  }
  const std::string::size_type slash = path.find_last_of('/');
  const std::string directory = slash == std::string::npos ? "." : path.substr(0, slash);
  const int dir_fd = open(directory.c_str(), O_RDONLY | O_DIRECTORY);
  if (dir_fd < 0) return false;
  const bool durable = fsync(dir_fd) == 0;
  const bool dir_closed = close(dir_fd) == 0;
  return durable && dir_closed;
}

}  // namespace patrolbot
#endif
