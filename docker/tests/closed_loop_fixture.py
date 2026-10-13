"""Test-only fake-SBC feedback plant; use only with network-none Nav2 tests.

Nav2 supplies DRIVE commands. This fixture supplies odometry and raycast scans,
never a controller or a replacement for Nav2 planner/controller acceptance.
"""
import math
from pathlib import Path
import sys
import time

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'diagnostics/localization'))
from geometry import MapRaster, integrate
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros2_ws/src/patrolbot_bridge/test'))
from test_fake_sbc_integration import FakeSbc


class ClosedLoopFakeSbc(FakeSbc):
    def __init__(self, yaml_path, map_id, revision, pose, laser=(.037,0,math.pi), beams=361):
        self.raster=MapRaster(yaml_path,map_id,revision)
        self.world_pose=list(pose)
        self.start_pose=list(pose)
        self.velocity=(0.,0.)
        self.last_step=time.monotonic()
        self.last_command=self.last_step
        self.distance=0.
        self.samples=[]
        self.laser=laser
        self.beams=beams
        self.motion_frozen=False
        self.scan_delay=0.
        self.history=[]
        super().__init__()
        self.charge_state=0
        self.dock_state='UNDOCKED'
        self.motors_enabled=True
        self.estop_pressed=False

    def _reply(self,connection,line):
        fields=line.split(':')
        if fields[0]=='DRIVE' and len(fields)==3:
            velocity=tuple(map(float,fields[1:]))
            if not all(math.isfinite(v) for v in velocity): raise ValueError('nonfinite DRIVE')
            with self._lock:
                self.velocity=velocity
                self.last_command=time.monotonic()
        super()._reply(connection,line)

    def _telemetry(self):
        now=time.monotonic()
        with self._lock:
            dt=min(now-self.last_step,.1)
            self.last_step=now
            velocity=self.velocity if now-self.last_command<.3 and self.motors_enabled and not self.estop_pressed else (0.,0.)
            if self.motion_frozen: velocity=(0.,0.)
            candidate=integrate(self.world_pose,*velocity,dt)
            if self.raster.cell(candidate[0],candidate[1])=='free':
                self.distance+=math.hypot(candidate[0]-self.world_pose[0],candidate[1]-self.world_pose[1])
                self.world_pose=candidate
            x,y,yaw=self.world_pose
            sx,sy,syaw=self.start_pose
            dx=x-sx;dy=y-sy
            self.odom_pose=(math.cos(syaw)*dx+math.sin(syaw)*dy,-math.sin(syaw)*dx+math.cos(syaw)*dy,yaw-syaw)
            sample={'time':now,'world_pose':list(self.world_pose),'odom':list(self.odom_pose),'velocity':list(velocity)}
            self.samples.append(sample)
            self.history.append((now,list(self.world_pose)))
            while len(self.history)>1 and self.history[1][0]<=now-self.scan_delay: self.history.pop(0)
            scan_pose=self.history[0][1]
        x,y,yaw=scan_pose
        lx,ly,roll=self.laser
        origin=(x+math.cos(yaw)*lx-math.sin(yaw)*ly,y+math.sin(yaw)*lx+math.cos(yaw)*ly)
        ranges=[]
        for i in range(self.beams):
            angle=-math.pi/2+i*math.pi/(self.beams-1)
            # Production laser static transform has a pi roll, reversing scan Y.
            hit=self.raster.raycast(origin,yaw+(-angle if abs(roll-math.pi)<1e-5 else angle),15.)
            ranges.append(str(hit['distance']) if hit['classification']=='occupied' else 'nan')
        with self._lock:
            # Both wire fields describe the same acquisition, as on the SBC.
            self.lidar_scan_age_ms=round(self.scan_delay*1000)
            self.laser_field=','.join(ranges)+'|LASER_AGE_MS:'+str(self.lidar_scan_age_ms)
        wire=super()._telemetry()
        return wire.replace(b',0,0|LASER:',f',{velocity[0]},{velocity[1]}|LASER:'.encode(),1)

    def snapshot(self):
        with self._lock:
            return {'map':self.raster.identity,'world_pose':list(self.world_pose),'distance_m':self.distance,
                    'samples':list(self.samples),'commands':list(self.received),'scan_delay':self.scan_delay,
                    'odom_epoch':self.odom_epoch}
