// Offline tests for the complete-scan freshness contract.

#include "../scan_timing.hpp"

#include <cstdio>
#include <vector>

namespace {

const char* g_case = "<none>";
int g_failures = 0;
int g_checks = 0;

#define CHECK_EQ(actual, expected)                                           \
  do {                                                                       \
    ++g_checks;                                                              \
    const long actual_ = (actual);                                           \
    const long expected_ = (expected);                                       \
    if (actual_ != expected_) {                                              \
      std::fprintf(stderr, "FAIL [%s] %s:%d: %s == %s (%ld vs %ld)\n",      \
                   g_case, __FILE__, __LINE__, #actual, #expected,           \
                   actual_, expected_);                                      \
      ++g_failures;                                                           \
    }                                                                         \
  } while (0)

#define RUN(name)                                                             \
  do {                                                                        \
    g_case = #name;                                                           \
    name();                                                                   \
  } while (0)

using patrolbot::oldestScanAgeMs;

void test_empty_scan_is_invalid() {
  CHECK_EQ(oldestScanAgeMs(std::vector<long>()), -1);
}

void test_zero_age_is_valid() {
  CHECK_EQ(oldestScanAgeMs(std::vector<long>{0, 0}), 0);
}

void test_single_beam_cannot_establish_scan_geometry() {
  CHECK_EQ(oldestScanAgeMs(std::vector<long>{0}), -1);
}

void test_boundary_age_is_valid() {
  CHECK_EQ(oldestScanAgeMs(std::vector<long>{1, 500, 12}), 500);
}

void test_mixed_ages_return_oldest() {
  CHECK_EQ(oldestScanAgeMs(std::vector<long>{40, 7, 312, 88}), 312);
}

void test_negative_age_is_invalid() {
  CHECK_EQ(oldestScanAgeMs(std::vector<long>{20, -1, 30}), -1);
}

void test_over_boundary_age_is_invalid() {
  CHECK_EQ(oldestScanAgeMs(std::vector<long>{499, 501}), -1);
}

void test_snapshot_processing_time_is_added() {
  CHECK_EQ(patrolbot::elapsedScanAgeMs(400, 100), 500);
  CHECK_EQ(patrolbot::elapsedScanAgeMs(400, 101), -1);
  CHECK_EQ(patrolbot::elapsedScanAgeMs(-1, 0), -1);
  CHECK_EQ(patrolbot::elapsedScanAgeMs(0, -1), -1);
  CHECK_EQ(patrolbot::elapsedScanAgeMs(0, 9223372036854775807LL), -1);
}

}  // namespace

int main() {
  RUN(test_empty_scan_is_invalid);
  RUN(test_zero_age_is_valid);
  RUN(test_single_beam_cannot_establish_scan_geometry);
  RUN(test_boundary_age_is_valid);
  RUN(test_mixed_ages_return_oldest);
  RUN(test_negative_age_is_invalid);
  RUN(test_over_boundary_age_is_invalid);
  RUN(test_snapshot_processing_time_is_added);

  std::printf("%d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
