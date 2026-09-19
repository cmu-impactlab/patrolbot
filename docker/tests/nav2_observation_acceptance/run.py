import json
import os
import tempfile
import subprocess
import time
start = time.monotonic()
with tempfile.TemporaryFile(mode='w+') as output:
    child = subprocess.Popen(['/tmp/observation-build/observation_acceptance'], stdout=output, stderr=subprocess.STDOUT)
    while True:
        pid, status, usage = os.wait4(child.pid, os.WNOHANG)
        if pid:
            child.returncode = os.waitstatus_to_exitcode(status)
            break
        if time.monotonic() - start > 30:
            child.kill()
            os.wait4(child.pid, 0)
            raise TimeoutError('observation acceptance exceeded 30 seconds')
        time.sleep(.01)
    output.seek(0)
    text = output.read()
print(text)
assert child.returncode == 0, child.returncode
assert text.count('has not been updated') == 1
assert text.count('is current again') == 1
assert 'after 144000 stale checks' in text
assert '[ERROR]' not in text
print(json.dumps({'wall_s': time.monotonic()-start, 'user_cpu_s': usage.ru_utime,
                  'system_cpu_s': usage.ru_stime, 'peak_rss_kib': usage.ru_maxrss,
                  'captured_log_bytes': len(text.encode()), 'stale_checks': 144000,
                  'simulated_outage_s': 28800, 'note': 'accelerated ROS time; steady logging cadence tested separately'}))
