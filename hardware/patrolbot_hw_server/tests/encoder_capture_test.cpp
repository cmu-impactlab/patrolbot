#include "../encoder_capture.hpp"
#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>

int main() {
  using patrolbot::EncoderCapture;
  using patrolbot::SipPoseSnapshot;
  const std::uint8_t payload[] = {0xfe,0xff,0xff,0xff,0x78,0x56,0x34,0x12};
  SipPoseSnapshot sip;
  sip.x_m = 1.; sip.y_m = 2.; sip.yaw_rad = .3;
  EncoderCapture capture;
  assert(!capture.recordEncoder(payload,8,100,sip));
  assert(!capture.start("origin",0,1,0,false));
  assert(!capture.start("",2,1000,0,true));
  assert(capture.start("controller-epoch-1",2,1000,100,true));
  assert(!capture.recordEncoder(payload,8,101,sip)); // no preceding SIP
  capture.observeSip(102);
  assert(!capture.recordEncoder(payload,7,103,sip));
  assert(capture.recordEncoder(payload,8,104,sip));
  assert(capture.rows().size()==1);
  const auto& row=capture.rows().front();
  assert(row.left_counts==4294967294ULL && row.right_counts==0x12345678);
  assert(row.captured_at_ms==104 && row.sip_captured_at_ms==102);
  assert(row.origin=="controller-epoch-1" && row.sip_x_m==1.);
  assert(!capture.recordEncoder(payload,8,104,sip)); // no advancing capture time
  sip.x_m=std::numeric_limits<double>::quiet_NaN();
  assert(!capture.recordEncoder(payload,8,105,sip));
  sip.x_m=1.;
  assert(capture.recordEncoder(payload,8,106,sip));
  assert(!capture.active(107)); // sample budget consumed
  assert(!capture.recordEncoder(payload,8,108,sip));
  assert(capture.start("controller-epoch-2",10,100,200,true));
  capture.observeSip(201);
  assert(!capture.recordEncoder(payload,8,502,sip)); // stale SIP
  assert(!capture.active(502)); // elapsed budget exhausted even without encoders
  assert(capture.rows().empty());
  assert(capture.start("controller-epoch-3",10,1000,600,true));
  capture.observeSip(601);
  assert(!capture.recordEncoder(payload,8,902,sip)); // >300ms SIP age
  assert(capture.active(902));
  capture.observeSip(903);
  assert(capture.recordEncoder(payload,8,904,sip));
  capture.stop();
  assert(!capture.active(905));
  assert(capture.rows().size()==1);
  assert(!capture.start("controller-epoch-4",4097,1000,1000,true));
  assert(!capture.start("controller-epoch-4",10,60001,1000,true));
}
