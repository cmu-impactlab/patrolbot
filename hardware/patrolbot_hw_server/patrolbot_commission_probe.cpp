// PatrolBot commissioning probe.
//
// Fills in the hardware facts that the SBC-owned undock operation refuses to run
// without: rear sonar transducer indices and their behaviour, E-stop signal
// semantics, idle fault and stall flags, the bumper-mask regression, and the
// timing of a charger release.
//
// DEFAULT MODE IS STRICTLY PASSIVE. It reads ARIA telemetry and writes CSV. It
// never sends a velocity command, never enables or disables motors, and never
// sends a charging command.
//
// --allow-single-charge-release IS AN ACTIVE MODE, NOT A READ-ONLY ONE. It may
// issue exactly one uARCS CHARGE 0 after a logged baseline, and only when all
// four preconditions hold: reported motion is stably zero, the motors are
// already disabled, the confirmation token is supplied, and no production
// patrolbot_server is running. There is no motor-enable and no motion path in
// any mode, and CHARGE 1 is never sent.
//
// SINGLE-OWNER CONSTRAINT: socat runs without fork and only one process may hold
// the base connection, so this tool CANNOT run while patrolbot_server is active.
// See docs/packages/patrolbot_hw_server.md for the stop-probe-restart procedure.

#include <dirent.h>
#include <errno.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cmath>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <string>
#include <vector>

#include "Aria.h"

namespace {

const char* kConfirmationToken = "RELEASE-CHARGE-ONCE";
const unsigned char kUarcsChargeCommand = 68;
const int kServerPort = 7272;

struct Options {
  Options()
      : seconds(60.0), rate_hz(20.0), allow_charge_release(false),
        confirmed(false), output_path("") {}

  double seconds;
  double rate_hz;
  bool allow_charge_release;
  bool confirmed;
  std::string output_path;
};

void usage() {
  std::fprintf(stderr,
      "patrolbot_commission_probe [--seconds=N] [--rate=HZ] [--out=PATH]\n"
      "                           [--allow-single-charge-release --confirm=%s]\n"
      "\n"
      "Default mode is passive: it only reads telemetry and writes CSV.\n"
      "--allow-single-charge-release is an ACTIVE mode. It sends exactly one\n"
      "uARCS CHARGE 0 and requires the confirmation token, stable zero motion,\n"
      "motors already disabled, and no running patrolbot_server.\n"
      "It never enables motors, never commands velocity, never sends CHARGE 1.\n",
      kConfirmationToken);
}

bool parseOptions(int argc, char** argv, Options* options) {
  for (int i = 1; i < argc; ++i) {
    const std::string argument(argv[i]);
    if (argument.compare(0, 10, "--seconds=") == 0) {
      options->seconds = std::atof(argument.c_str() + 10);
    } else if (argument.compare(0, 7, "--rate=") == 0) {
      options->rate_hz = std::atof(argument.c_str() + 7);
    } else if (argument.compare(0, 6, "--out=") == 0) {
      options->output_path = argument.substr(6);
    } else if (argument == "--allow-single-charge-release") {
      options->allow_charge_release = true;
    } else if (argument.compare(0, 10, "--confirm=") == 0) {
      options->confirmed = argument.substr(10) == kConfirmationToken;
    } else if (argument == "--help" || argument == "-h") {
      return false;
    } else {
      // Anything else is left for ArArgumentParser (-rh, -rrtp, ...).
      continue;
    }
  }
  if (options->seconds <= 0.0 || options->seconds > 3600.0) return false;
  if (options->rate_hz < 1.0 || options->rate_hz > 50.0) return false;
  return true;
}

// The base connection is single-owner. Binding the server port is a reliable,
// unprivileged way to prove the production server is not holding it.
bool serverPortIsFree() {
  const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) return false;
  int one = 1;
  ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  struct sockaddr_in address;
  ::memset(&address, 0, sizeof(address));
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_ANY);
  address.sin_port = htons(kServerPort);
  const int result =
      ::bind(fd, reinterpret_cast<struct sockaddr*>(&address), sizeof(address));
  ::close(fd);
  return result == 0;
}

// Belt and braces: a server that has not yet bound, or has just crashed, is
// still caught by its process name.
bool serverProcessRunning() {
  DIR* proc = ::opendir("/proc");
  if (proc == NULL) return false;
  bool found = false;
  struct dirent* entry = NULL;
  const pid_t self = ::getpid();
  while (!found && (entry = ::readdir(proc)) != NULL) {
    const int pid = std::atoi(entry->d_name);
    if (pid <= 0 || pid == self) continue;
    char path[64];
    std::snprintf(path, sizeof(path), "/proc/%d/comm", pid);
    FILE* comm = std::fopen(path, "r");
    if (comm == NULL) continue;
    char name[128];
    if (std::fgets(name, sizeof(name), comm) != NULL) {
      // Linux truncates comm to 15 characters: "patrolbot_serve".
      if (std::strncmp(name, "patrolbot_serve", 15) == 0) found = true;
    }
    std::fclose(comm);
  }
  ::closedir(proc);
  return found;
}

struct Sample {
  int charge_state;
  bool motors_enabled;
  bool estop_pressed;
  unsigned int flags;
  unsigned int fault_flags;
  unsigned int stall_value;
  double enc_x_mm;
  double enc_y_mm;
  double enc_th_deg;
  double odometer_m;
  double vel;
  double rot_vel;
  double battery_volts;
  double state_of_charge;
  int temperature;
};

Sample readSample(ArRobot* robot) {
  Sample sample;
  robot->lock();
  sample.charge_state = robot->getChargeState();
  sample.motors_enabled = robot->areMotorsEnabled();
  sample.estop_pressed = robot->isEStopPressed();
  sample.flags = static_cast<unsigned int>(robot->getFlags());
  sample.fault_flags = static_cast<unsigned int>(robot->getFaultFlags());
  sample.stall_value = static_cast<unsigned int>(robot->getStallValue());
  const ArPose pose = robot->getEncoderPose();
  sample.enc_x_mm = pose.getX();
  sample.enc_y_mm = pose.getY();
  sample.enc_th_deg = pose.getTh();
  sample.odometer_m = robot->getOdometerDistanceMeters();
  sample.vel = robot->getVel() / 1000.0;
  sample.rot_vel = robot->getRotVel();
  sample.battery_volts = robot->getRealBatteryVoltageNow();
  sample.state_of_charge =
      robot->haveStateOfCharge() ? robot->getStateOfCharge() : -1.0;
  sample.temperature = static_cast<int>(robot->getTemperature());
  robot->unlock();
  return sample;
}

bool stablyStationary(const std::vector<Sample>& history, std::size_t required) {
  if (history.size() < required) return false;
  for (std::size_t i = history.size() - required; i < history.size(); ++i) {
    if (std::fabs(history[i].vel) > 0.005) return false;
    if (std::fabs(history[i].rot_vel) > 0.6) return false;  // degrees per second
  }
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  Options options;
  if (!parseOptions(argc, argv, &options)) {
    usage();
    return 2;
  }

  if (options.allow_charge_release && !options.confirmed) {
    std::fprintf(stderr,
                 "REFUSED: --allow-single-charge-release requires --confirm=%s\n"
                 "This mode is NOT read-only: it sends one uARCS CHARGE 0.\n",
                 kConfirmationToken);
    return 3;
  }

  // Checked before ARIA opens the base connection, so a running server is never
  // disturbed by a second client attaching to the same single-owner port.
  if (serverProcessRunning() || !serverPortIsFree()) {
    std::fprintf(stderr,
                 "REFUSED: patrolbot_server appears to be running. The base "
                 "connection is single-owner.\nStop the service first; see "
                 "docs/packages/patrolbot_hw_server.md.\n");
    return 4;
  }

  Aria::init();
  ArArgumentParser parser(&argc, argv);
  parser.loadDefaultArguments();
  ArRobot robot;
  ArRobotConnector connector(&parser, &robot);
  if (!connector.parseArgs()) {
    std::fprintf(stderr, "REFUSED: could not parse robot connection arguments\n");
    Aria::exit(5);
  }
  if (!connector.connectRobot()) {
    std::fprintf(stderr, "REFUSED: could not connect to the local robot base\n");
    Aria::exit(5);
  }
  robot.runAsync(true);
  robot.lock();
  // Sonar is a sensor, not an actuator. Motors are deliberately left in
  // whatever state the hardware is already in; this tool never changes them.
  robot.enableSonar();
  robot.unlock();

  std::string path = options.output_path;
  if (path.empty()) {
    char generated[128];
    std::snprintf(generated, sizeof(generated),
                  "patrolbot_commission_%ld.csv",
                  static_cast<long>(std::time(NULL)));
    path = generated;
  }
  FILE* out = std::fopen(path.c_str(), "w");
  if (out == NULL) {
    std::fprintf(stderr, "REFUSED: cannot write %s: %s\n", path.c_str(),
                 std::strerror(errno));
    Aria::exit(6);
  }

  robot.lock();
  const int sonar_count = robot.getNumSonar();
  robot.unlock();

  // Sonar geometry is logged once, in the header. This is what identifies which
  // ARIA indices are the rear transducers.
  std::fprintf(out, "# patrolbot_commission_probe\n");
  std::fprintf(out, "# mode=%s\n",
               options.allow_charge_release ? "ACTIVE_SINGLE_CHARGE_RELEASE"
                                            : "PASSIVE");
  std::fprintf(out, "# sonar_count=%d\n", sonar_count);
  for (int i = 0; i < sonar_count; ++i) {
    robot.lock();
    ArSensorReading* reading = robot.getSonarReading(i);
    if (reading != NULL) {
      std::fprintf(out, "# sonar[%d] x_mm=%.1f y_mm=%.1f th_deg=%.1f\n", i,
                   reading->getSensorX(), reading->getSensorY(),
                   reading->getSensorTh());
    } else {
      std::fprintf(out, "# sonar[%d] unavailable\n", i);
    }
    robot.unlock();
  }
  std::fprintf(out,
               "elapsed_ms,event,charge_state,motors_enabled,estop_pressed,"
               "flags,fault_flags,stall_value,front_bits,rear_bits,left_stall,"
               "right_stall,enc_x_mm,enc_y_mm,enc_th_deg,odometer_m,vel_mps,"
               "rotvel_dps,battery_v,soc,temp_c");
  for (int i = 0; i < sonar_count; ++i) {
    std::fprintf(out, ",sonar%d_mm,sonar%d_age_ms", i, i);
  }
  std::fprintf(out, "\n");

  const long period_ms = static_cast<long>(1000.0 / options.rate_hz);
  const long total_ms = static_cast<long>(options.seconds * 1000.0);
  // A baseline is always logged before any active step, so the release timing
  // can be read against known-good prior state.
  const long baseline_ms = options.allow_charge_release ? 5000 : 0;

  ArTime started;
  started.setToNow();
  std::vector<Sample> history;
  bool release_sent = false;
  long release_sent_at_ms = -1;
  bool release_refused_logged = false;

  while (robot.isRunning() && started.mSecSince() < total_ms) {
    const long elapsed = started.mSecSince();
    const Sample sample = readSample(&robot);
    history.push_back(sample);
    if (history.size() > 200) history.erase(history.begin());

    const char* event = "sample";

    if (options.allow_charge_release && !release_sent && elapsed >= baseline_ms) {
      const bool stationary = stablyStationary(history, 20);
      const bool motors_off = !sample.motors_enabled;
      if (stationary && motors_off) {
        robot.lock();
        // Exactly one CHARGE 0, for the whole run. Nothing else is ever sent.
        robot.comInt(kUarcsChargeCommand, 0);
        robot.unlock();
        release_sent = true;
        release_sent_at_ms = elapsed;
        event = "charge_release_sent";
        std::fprintf(stderr, "CHARGE 0 sent once at %ld ms\n", elapsed);
      } else if (!release_refused_logged && elapsed > baseline_ms + 5000) {
        release_refused_logged = true;
        event = "charge_release_refused_preconditions";
        std::fprintf(stderr,
                     "CHARGE 0 NOT sent: stationary=%d motors_disabled=%d\n",
                     stationary ? 1 : 0, motors_off ? 1 : 0);
      }
    }

    std::fprintf(out,
                 "%ld,%s,%d,%d,%d,%u,%u,%u,%u,%u,%d,%d,"
                 "%.1f,%.1f,%.2f,%.4f,%.4f,%.2f,%.2f,%.1f,%d",
                 elapsed, event, sample.charge_state,
                 sample.motors_enabled ? 1 : 0, sample.estop_pressed ? 1 : 0,
                 sample.flags, sample.fault_flags, sample.stall_value,
                 (sample.stall_value & 0x7e00u) >> 9,
                 (sample.stall_value & 0x007eu) >> 1,
                 (sample.stall_value & 0x0100u) ? 1 : 0,
                 (sample.stall_value & 0x0001u) ? 1 : 0,
                 sample.enc_x_mm, sample.enc_y_mm, sample.enc_th_deg,
                 sample.odometer_m, sample.vel, sample.rot_vel,
                 sample.battery_volts, sample.state_of_charge,
                 sample.temperature);

    for (int i = 0; i < sonar_count; ++i) {
      robot.lock();
      ArSensorReading* reading = robot.getSonarReading(i);
      if (reading != NULL) {
        // Age is what distinguishes "no echo" from "sensor not reporting".
        std::fprintf(out, ",%u,%ld", reading->getRange(),
                     reading->getTimeTaken().mSecSince());
      } else {
        std::fprintf(out, ",,");
      }
      robot.unlock();
    }
    std::fprintf(out, "\n");
    std::fflush(out);

    ArUtil::sleep(period_ms);
  }

  std::fprintf(stderr, "wrote %s (%d sonar channels)\n", path.c_str(),
               sonar_count);
  if (options.allow_charge_release) {
    if (release_sent) {
      std::fprintf(stderr,
                   "one CHARGE 0 was sent at %ld ms; read the charge_state "
                   "column after that row for release and re-assert timing\n",
                   release_sent_at_ms);
    } else {
      std::fprintf(stderr, "no CHARGE 0 was sent: preconditions never held\n");
    }
  }
  std::fclose(out);
  // The probe leaves motors, charging and velocity exactly as it found them.
  Aria::exit(0);
  return 0;
}
