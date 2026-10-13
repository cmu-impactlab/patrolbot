"""Exercise plant feedback without importing ROS or opening a socket."""
import importlib.util
import math
from pathlib import Path
import sys
import tempfile
import threading
import time
import types
import unittest

fake_module=types.ModuleType('test_fake_sbc_integration')
class FakeSbc:
    def __init__(self):
        self._lock=threading.Lock();self.received=[];self.odom_epoch='fixture'
        self.lidar_scan_age_ms=0
    def _reply(self,connection,line): pass
    def _telemetry(self):
        return (f'ODOM:{self.odom_pose[0]},{self.odom_pose[1]},{self.odom_pose[2]},0,0|LASER:'+self.laser_field).encode()
fake_module.FakeSbc=FakeSbc
sys.modules['test_fake_sbc_integration']=fake_module
spec=importlib.util.spec_from_file_location('fixture',Path(__file__).with_name('closed_loop_fixture.py'))
fixture=importlib.util.module_from_spec(spec);spec.loader.exec_module(fixture)

class PlantTests(unittest.TestCase):
    def test_drive_feedback_and_frozen_progress(self):
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory)
            pixels=bytes([0 if x in (0,19) or y in (0,19) else 255 for y in range(20) for x in range(20)])
            (p/'map.pgm').write_bytes(b'P5\n20 20\n255\n'+pixels)
            (p/'map.yaml').write_text('image: map.pgm\nresolution: 0.1\norigin: [0,0,0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
            plant=fixture.ClosedLoopFakeSbc(p/'map.yaml','test','r1',[1,1,0],beams=9)
            plant._reply(None,'DRIVE:0.2:0.1');plant.last_step=time.monotonic()-.1
            wire=plant._telemetry()
            self.assertGreater(plant.world_pose[0],1)
            self.assertIn(b',0.2,0.1|LASER:',wire)
            self.assertGreater(plant.snapshot()['distance_m'],0)
            before=list(plant.world_pose);plant.motion_frozen=True
            plant.last_step=time.monotonic()-.1
            self.assertIn(b',0.0,0.0|LASER:',plant._telemetry())
            self.assertEqual(plant.world_pose,before)
            plant.motion_frozen=False;plant.last_command=time.monotonic()-1
            self.assertIn(b',0.0,0.0|LASER:',plant._telemetry())
            plant.scan_delay=2.
            wire=plant._telemetry()
            self.assertIn(b'LASER_AGE_MS:2000',wire)
            self.assertEqual(plant.lidar_scan_age_ms,2000,
                'AUX LIDAR age must agree with LASER acquisition age')
            plant.scan_delay=0.
            plant._telemetry()
            self.assertEqual(plant.lidar_scan_age_ms,0)

if __name__=='__main__':unittest.main()
