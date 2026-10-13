#ifndef PATROLBOT_ENCODER_CAPTURE_HPP
#define PATROLBOT_ENCODER_CAPTURE_HPP
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace patrolbot {
struct SipPoseSnapshot { double x_m=0, y_m=0, yaw_rad=0; };
struct EncoderCaptureRow {
  std::string origin;
  std::uint32_t left_counts, right_counts;
  long long captured_at_ms, sip_captured_at_ms;
  double sip_x_m, sip_y_m, sip_yaw_rad;
};
class EncoderCapture {
 public:
  bool start(const std::string& origin, unsigned count, long long duration_ms,
             long long now_ms, bool packet_format_verified) {
    if (!packet_format_verified || origin.empty() || count==0 || count>4096 ||
        duration_ms<=0 || duration_ms>60000 || now_ms<0) return false;
    origin_=origin; limit_=count; deadline_=now_ms+duration_ms; sip_at_=-1;
    previous_=-1; enabled_=true; rows_.clear(); return true;
  }
  void stop() { enabled_=false; }
  bool active(long long now) const {
    return enabled_ && now<deadline_ && rows_.size()<limit_;
  }
  void observeSip(long long now) { sip_at_=now; }
  bool recordEncoder(const std::uint8_t* bytes, unsigned size, long long now,
                     const SipPoseSnapshot& sip) {
    if (!active(now) || size!=8 || !bytes || sip_at_<0 || now<sip_at_ ||
        now-sip_at_>300 || now<=previous_ || !std::isfinite(sip.x_m) ||
        !std::isfinite(sip.y_m) || !std::isfinite(sip.yaw_rad)) return false;
    EncoderCaptureRow row={origin_,decode(bytes),decode(bytes+4),now,sip_at_,
                           sip.x_m,sip.y_m,sip.yaw_rad};
    rows_.push_back(row); previous_=now; return true;
  }
  const std::vector<EncoderCaptureRow>& rows() const { return rows_; }
 private:
  static std::uint32_t decode(const std::uint8_t* p) {
    return std::uint32_t(p[0]) | (std::uint32_t(p[1])<<8) |
           (std::uint32_t(p[2])<<16) | (std::uint32_t(p[3])<<24);
  }
  bool enabled_=false;
  unsigned limit_=0;
  long long deadline_=0, sip_at_=-1, previous_=-1;
  std::string origin_;
  std::vector<EncoderCaptureRow> rows_;
};
}
#endif
