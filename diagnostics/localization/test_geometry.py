import importlib.util
import json
import sys
from unittest.mock import patch
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location('geometry', Path(__file__).with_name('geometry.py'))
g = importlib.util.module_from_spec(spec)
spec.loader.exec_module(g)

class GeometryTests(unittest.TestCase):
    def scene(self, directory):
        p = Path(directory)
        # Top raster row is high map Y. Unknown cell blocks visibility.
        (p/'map.pgm').write_bytes(b'P5\n5 5\n255\n'+bytes([0]*5+[0,255,255,255,0]*3+[0]*5))
        (p/'map.yaml').write_text('image: map.pgm\nresolution: 0.1\norigin: [2, 3, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
        return g.MapRaster(p/'map.yaml', 'fixture', 'revision')

    def test_metadata_and_first_surface(self):
        with tempfile.TemporaryDirectory() as d:
            m = self.scene(d)
            hit = m.raycast((2.25,3.25),0,1)
            self.assertEqual(hit['classification'],'occupied')
            self.assertAlmostEqual(hit['distance'],.15,places=6)
            self.assertEqual(m.cell(2.05,3.45),'occupied')
            self.assertEqual(m.cell(1.9,3),'out_of_map')

    def test_unknown_and_occlusion(self):
        with tempfile.TemporaryDirectory() as d:
            m = self.scene(d)
            self.assertEqual(m.compare((2.25,3.25),0,.35)['classification'],'occupied')
            m.pixels[2*5+3]=205
            self.assertEqual(m.raycast((2.25,3.25),0,1)['classification'],'unknown')

    def test_one_wall_inconclusive(self):
        with tempfile.TemporaryDirectory() as d:
            m = self.scene(d)
            scan={'angle_min':0,'angle_increment':0,'ranges':[.15]*3}
            result=g.analyze(m,scan,[2.25,3.25,0],[2.25,3.25,0])
            self.assertEqual(result['verdict'],'inconclusive')

    def test_returns_outside_declared_scan_range_are_not_wall_residuals(self):
        with tempfile.TemporaryDirectory() as d:
            m=self.scene(d)
            scan={'angle_min':0,'angle_increment':0,'ranges':[.15,32.763],
                  'range_min':.1,'range_max':8.}
            result=g.analyze(m,scan,[2.25,3.25,0],[2.25,3.25,0])
            self.assertEqual(len(result['overlays']['supplied']['hits']),1)
            self.assertEqual(result['overlays']['supplied']['excluded_ranges_count'],1)

    def test_replay_rejects_changed_map_files(self):
        for changed in ('map.pgm','map.yaml'):
            with self.subTest(changed=changed), tempfile.TemporaryDirectory() as d:
                p=Path(d);raster=self.scene(d)
                capture={'complete':True,'map':raster.identity,'scan':{'angle_min':0,'angle_increment':0,'ranges':[.15]},
                         'supplied_pose':[2.25,3.25,0],'amcl_pose':[2.25,3.25,0],'laser_transform':[0,0,0]}
                (p/'capture.json').write_text(json.dumps(capture))
                target=p/changed
                if changed.endswith('.pgm'):
                    data=bytearray(target.read_bytes());data[-1]=1;target.write_bytes(data)
                else: target.write_text(target.read_text().replace('resolution: 0.1','resolution: 0.2'))
                with patch.object(sys,'argv',['geometry',str(p/'capture.json'),'--output',str(p/'result.json')]): g.main()
                result=json.loads((p/'result.json').read_text())
                self.assertEqual(result['verdict'],'inconclusive')
                self.assertIn('map_identity_mismatch',result['missing'])

    def test_motion_is_nonzero_feedback(self):
        pose=g.integrate([0,0,0],.2,.5,1)
        self.assertGreater(pose[0],0)
        self.assertGreater(pose[1],0)
        self.assertAlmostEqual(pose[2],.5)


class EvidenceTests(unittest.TestCase):
    def test_missing_tf_and_delayed_sensor_fail_closed(self):
        result=g.check_evidence({'now_ns':2_000_000_000,'scan_stamp_ns':1,
            'odom_stamp_ns':1,'tf_edges':[['map','odom']], 'poses':[[0,0,0],[1,0,0]]})
        self.assertFalse(result['timing_ok'])
        self.assertFalse(result['tf_chain_ok'])
        self.assertGreater(result['spread_m'],.9)

    def test_timing_uses_existing_half_second_sensor_limit(self):
        result=g.check_evidence({'now_ns':1_400_000_000,'scan_stamp_ns':1_000_000_000,
            'odom_stamp_ns':1_000_000_000,'amcl_stamp_ns':1_000_000_000})
        self.assertTrue(result['timing_ok'])
        result=g.check_evidence({'now_ns':1_501_000_000,'scan_stamp_ns':1_000_000_000,
            'odom_stamp_ns':1_000_000_000,'amcl_stamp_ns':1_000_000_000})
        self.assertFalse(result['timing_ok'])

    def test_fresh_scan_does_not_mask_old_amcl_pose(self):
        result=g.check_evidence({'now_ns':100_100_000_000,'scan_stamp_ns':100_000_000_000,
            'odom_stamp_ns':100_000_000_000,'amcl_stamp_ns':2_000_000_000})
        self.assertFalse(result['timing_ok'])

class StructuralTests(unittest.TestCase):
    def room(self, directory):
        p=Path(directory)
        size=101
        (p/'room.pgm').write_bytes(b'P5\n101 101\n255\n'+bytes(
            0 if x in (0,size-1) or y in (0,size-1) else 255
            for y in range(size) for x in range(size)))
        (p/'room.yaml').write_text('image: room.pgm\nresolution: 0.05\norigin: [0, 0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
        return g.MapRaster(p/'room.yaml','synthetic-room','v1')

    def scan(self, raster, angle=0):
        import math
        minimum=-math.pi/6
        increment=math.pi/180
        return {'angle_min':minimum,'angle_increment':increment,'ranges':[
            raster.raycast((2.5,2.5),minimum+i*increment+angle,10)['distance']
            for i in range(121)]}

    def test_nearer_furniture_return_is_occluded(self):
        with tempfile.TemporaryDirectory() as d:
            m=self.room(d)
            self.assertEqual(m.compare((2.5,2.5),0,.2)['classification'],'occluded')
            self.assertEqual(m.compare((2.5,2.5),0,3.)['classification'],'occupied')

    def test_seed_displacement_fails_even_without_second_wall(self):
        with tempfile.TemporaryDirectory() as d:
            m=self.room(d)
            result=g.analyze(m,{'angle_min':0,'angle_increment':0,'ranges':[2.5]},
                [2.5,2.5,0],[2.39,2.5,0])
            self.assertEqual(result['verdict'],'fail')

    def test_two_structural_walls_pass_with_fitted_corner(self):
        with tempfile.TemporaryDirectory() as d:
            m=self.room(d)
            result=g.analyze(m,self.scan(m),[2.5,2.5,0],[2.5,2.5,0])
            self.assertEqual(result['verdict'],'pass')
            self.assertLess(result['overlays']['amcl']['geometry']['corner_error_m'],.01)
            self.assertEqual(len(result['overlays']['amcl']['geometry']['walls']),2)
            self.assertTrue(all(w['p90_range_residual_m']<.01 for w in result['overlays']['amcl']['geometry']['walls']))
            self.assertFalse(result['physical_acceptance'])

    def test_scan_heading_error_fails_even_when_pose_yaws_equal(self):
        import math
        with tempfile.TemporaryDirectory() as d:
            m=self.room(d)
            result=g.analyze(m,self.scan(m,math.radians(8)),[2.5,2.5,0],[2.5,2.5,0])
            self.assertEqual(result['verdict'],'fail')
            geometry=result['overlays']['amcl']['geometry']
            self.assertGreater(max(w['normal_angle_error_deg'] for w in geometry['walls']),3)
            self.assertGreater(geometry['corner_error_m'],.15)

    def test_corner_displacement_fails(self):
        with tempfile.TemporaryDirectory() as d:
            m=self.room(d)
            result=g.analyze(m,self.scan(m),[2.5,2.5,0],[2.3,2.3,0])
            self.assertEqual(result['verdict'],'fail')
            self.assertGreater(result['displacement_m'],.10)

    def test_same_wall_has_no_corner(self):
        import math
        with tempfile.TemporaryDirectory() as d:
            m=self.room(d)
            scan={'angle_min':-.2,'angle_increment':.01,'ranges':[
                m.raycast((2.5,2.5),-.2+i*.01,10)['distance'] for i in range(41)]}
            result=g.analyze(m,scan,[2.5,2.5,0],[2.5,2.5,0])
            self.assertEqual(result['verdict'],'inconclusive')
            self.assertEqual(len(result['overlays']['amcl']['geometry']['walls']),1)

    def test_axis_crossings_without_wall_span_inconclusive(self):
        with tempfile.TemporaryDirectory() as d:
            m=self.room(d)
            result=g.analyze(m,{'angle_min':0,'angle_increment':1.5707963267948966,
                'ranges':[2.5,2.5]},[2.5,2.5,0],[2.5,2.5,0])
            self.assertEqual(result['verdict'],'inconclusive')

    def test_tf_numerical_composition_and_time(self):
        import math
        evidence={'now_ns':1_100_000_000,'scan_stamp_ns':1_000_000_000,
            'odom_stamp_ns':1_000_000_000,'amcl_stamp_ns':1_000_000_000,'transforms':[
                {'parent':'map','child':'odom','stamp_ns':1_000_000_000,'pose':[1,2,math.pi/2]},
                {'parent':'odom','child':'base_link','stamp_ns':1_000_000_000,'pose':[2,0,0]},
                {'parent':'map','child':'base_link','stamp_ns':1_000_000_000,'pose':[1,4,math.pi/2]},
                {'parent':'base_link','child':'laser_frame','stamp_ns':1_000_000_000,'pose':[.2,0,0]}]}
        self.assertTrue(g.check_evidence(evidence)['tf_chain_ok'])
        evidence['transforms'][3].update(stamp_ns=0,static=True)
        self.assertTrue(g.check_evidence(evidence)['tf_chain_ok'])
        evidence['transforms'][3]['static']=False
        self.assertFalse(g.check_evidence(evidence)['tf_chain_ok'])
        evidence['transforms'][3]['static']=True
        evidence['transforms'][2]['pose'][0]+=.2
        self.assertFalse(g.check_evidence(evidence)['tf_chain_ok'])
        evidence['transforms'][2]['pose'][0]-=.2
        evidence['transforms'][1]['stamp_ns']-=1
        self.assertFalse(g.check_evidence(evidence)['tf_chain_ok'])
        evidence['transforms'][1]['stamp_ns']+=1
        evidence['now_ns']=2_000_000_000
        self.assertFalse(g.check_evidence(evidence)['tf_chain_ok'])
        evidence['now_ns']=1_100_000_000
        evidence['transforms'][2]['pose'][2]+=.1
        self.assertFalse(g.check_evidence(evidence)['tf_chain_ok'])

    def test_tf_edges_are_not_numeric_evidence(self):
        result=g.check_evidence({'now_ns':1,'scan_stamp_ns':1,'odom_stamp_ns':1,
            'tf_edges':[['map','odom'],['odom','base_link'],['base_link','laser_frame']]})
        self.assertFalse(result['tf_chain_ok'])

if __name__ == '__main__': unittest.main()


def test_disconnected_map_patches_do_not_create_a_wall(tmp_path):
    size=80;pixels=bytearray([255])*(size*size)
    for y in list(range(20,23))+list(range(40,43)):
        pixels[(size-1-y)*size+20]=0
    (tmp_path/'patches.pgm').write_bytes(b'P5\n80 80\n255\n'+pixels)
    (tmp_path/'patches.yaml').write_text('image: patches.pgm\nresolution: 0.05\norigin: [0,0,0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    raster=g.MapRaster(tmp_path/'patches.yaml','patches','v1')
    hits=[{'classification':'occupied','residual':0.,'map_endpoint':[1.,y+.018*i],
           'scan_endpoint':[1.,y+.018*i]} for y in (1.005,2.005) for i in range(8)]
    assert not g.structural_geometry(hits,raster)['walls']
