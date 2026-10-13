"""Offline regression using real Nav2 pf_init and the applied diagnostic block.

Pass a Nav2 1.3.12 source directory with the numbered patches applied.
This is not ROS or exact-image acceptance.
"""
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile

root = Path(sys.argv[1]) / 'nav2_amcl'
source = (root / 'src/amcl_node.cpp').read_text()
start = source.find('  // Observe the initialized particles before any sensor weighting.')
assert start >= 0, 'pf_init emits no applied seed diagnostic'
block = source[start:source.index('  // Acknowledge the original operator stamp', start)]
with tempfile.TemporaryDirectory(prefix='amcl-seed-test-') as directory:
    work = Path(directory)
    objects = []
    for name in ('pf', 'pf_kdtree', 'pf_pdf', 'pf_vector', 'eig3'):
        obj = work / (name + '.o')
        subprocess.run(['cc', '-DHAVE_DRAND48', '-I'+str(root/'include'), '-c',
                        str(root/'src/pf'/ (name+'.c')), '-o', str(obj)], check=True)
        objects.append(str(obj))
    harness = work/'check.cpp'
    harness.write_text('''#include <cmath>
#include <cstdio>
#include <cstdlib>
#include "nav2_amcl/pf/pf.hpp"
#define RCLCPP_INFO(logger, ...) std::printf(__VA_ARGS__); std::puts("")
struct Clock { long long nanoseconds() { return 2000000000LL; } };
Clock now() { return {}; }
int main() {
  pf_t * pf_ = pf_alloc(4000, 4000, 0., 0., nullptr);
  srand48(17);
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = 5.; pf_init_pose_mean.v[1] = 5.;
  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  pf_init_pose_cov.m[0][0] = pf_init_pose_cov.m[1][1] = .0009;
  pf_init_pose_cov.m[2][2] = .00030461742;
  const long long accepted_initial_pose_stamp_ns_ = 1000000000LL;
  pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);
''' + block + '''  pf_free(pf_);
}
''')
    binary = work/'check'
    subprocess.run(['c++', '-std=c++17', '-Wall', '-Wextra', '-Werror',
                    '-I'+str(root/'include'), str(harness), *objects, '-lm', '-o', str(binary)], check=True)
    output = subprocess.check_output([str(binary)], text=True)
    record = json.loads(output.split('Applied initial pose: ', 1)[1])
    assert record['request_stamp_ns'] == 1000000000
    assert record['application_stamp_ns'] == 2000000000
    assert record['mean'] == [5., 5., 0.]
    assert record['covariance'] == [.0009, 0., 0., 0., .0009, 0., 0., 0., .00030461742]
    assert math.hypot(record['sample_mean'][0]-5., record['sample_mean'][1]-5.) <= .01
    assert abs(record['sample_mean'][2]) <= math.radians(.3)
    assert record['within_envelope']/record['sample_count'] >= .99
    assert record['sample_count'] == 4000
    print(json.dumps(record, sort_keys=True))
