import importlib.util
import copy
import json
from pathlib import Path
import unittest

spec=importlib.util.spec_from_file_location('capture',Path(__file__).with_name('capture.py'))
c=importlib.util.module_from_spec(spec);spec.loader.exec_module(c)

def pose(x=1):
    return {'header':{'stamp':{'sec':1,'nanosec':0}},'pose':{'pose':{'position':{'x':x,'y':2},'orientation':{'x':0,'y':0,'z':0,'w':1}},'covariance':[.01]*36}}

class CaptureTests(unittest.TestCase):
    def test_missing_seed_is_explicit(self):
        result=c.convert([],{'map_id':'m','revision':'r','yaml':'map.yaml'})
        self.assertFalse(result['complete'])
        self.assertIn('seed',result['missing'])
        self.assertIn('exact_dashboard_request',result['missing'])

    def test_roll_reversal_and_no_invented_request_id(self):
        records=[{'topic':'/initialpose','message':pose()},
                 {'topic':'/amcl_pose','message':pose()},
                 {'topic':'/odom','message':pose()},
                 {'topic':'/scan','now_ns':1000000000,'transforms':[
                     {'parent':'base_link','child':'laser_frame','stamp_ns':1000000000,
                      'pose':[.037,0,0],'quaternion':[1,0,0,0]}],
                  'message':{'header':{'stamp':{'sec':1,'nanosec':0}},'angle_min':-1.,'angle_increment':.1,'ranges':[1,None],'range_min':.1,'range_max':8.}}]
        result=c.convert(records,{'map_id':'m','revision':'r','yaml':'map.yaml'})
        self.assertEqual(result['scan']['angle_increment'],-.1)
        self.assertEqual(result['scan']['angle_min'],1.)
        self.assertEqual(result['scan']['range_max'],8.)
        self.assertIsNone(result['identity']['command_id'])
        self.assertFalse(result['complete'])
        self.assertIn('accepted_seed',result['missing'])

    def test_exact_request_is_source_of_supplied_pose(self):
        request={'data':{'command_id':'request','command':'set_initial_pose',
            'goal':{'map_id':'m','map_revision':'r','x':8.,'y':9.,'yaw':3.14}}}
        records=[{'topic':'/initialpose','message':pose()},
            {'topic':'/scan','now_ns':1_000_000_000,'transforms':[],
             'message':{'header':{'stamp':{'sec':1,'nanosec':0}},
                        'angle_min':0,'angle_increment':.1,'ranges':[1.]}}]
        result=c.convert(records,{'map_id':'m','revision':'r','yaml':'map.yaml'},request)
        self.assertEqual(result['supplied_pose'],[8.,9.,3.14])
        self.assertIn('request_seed_mismatch',result['missing'])
        self.assertFalse(result['complete'])

    def complete_records(self, amcl_sec=100):
        seed=pose()
        estimate=pose();estimate['header']['stamp']['sec']=amcl_sec
        odom=pose();odom['header']['stamp']['sec']=100
        identity={'map_id':'m','revision':'r','yaml':'map.yaml'}
        request={'command':'set_initial_pose','goal':{'map_id':'m','map_revision':'r','x':1,'y':2,'yaw':0}}
        transforms=[{'parent':a,'child':b,'stamp_ns':100_000_000_000,
                     'pose':[0,0,0],'quaternion':[0,0,0,1]} for a,b in
                    [('map','odom'),('odom','base_link'),('base_link','laser_frame'),('map','base_link')]]
        records=[{'topic':'/initialpose','message':seed},
                 {'topic':'/amcl/accepted_initial_pose','message':seed},
                 {'topic':'/rosout','message':{'msg':'Applied initial pose: '+json.dumps({'request_stamp_ns':1_000_000_000})}},
                 {'topic':'/particle_cloud','message':{}},
                 {'topic':'/patrolbot/base_state','message':{'map_id':'m','map_revision':'r'}},
                 {'topic':'/amcl_pose','message':estimate},
                 {'topic':'/odom','message':odom},
                 {'topic':'/scan','now_ns':100_100_000_000,'transforms':transforms,
                  'message':{'header':{'stamp':{'sec':100,'nanosec':0}},'angle_min':0,'angle_increment':.1,'ranges':[1]}}]
        return records,identity,request

    def test_stale_amcl_with_fresh_scan_is_inconclusive(self):
        result=c.convert(*self.complete_records(amcl_sec=2))
        self.assertFalse(result['complete'])
        self.assertIn('amcl_scan_timestamp_mismatch',result['missing'])
        self.assertEqual(result['evidence']['amcl_stamp_ns'],2_000_000_000)

    def test_matching_amcl_scan_preserves_timestamp_evidence(self):
        result=c.convert(*self.complete_records())
        self.assertTrue(result['complete'],result['missing'])
        self.assertEqual(result['evidence']['amcl_stamp_ns'],result['evidence']['scan_stamp_ns'])

    def test_late_estimate_pairs_its_scan_even_after_newer_stationary_scans(self):
        records,identity,request=self.complete_records()
        estimate=records.pop(5)
        records.append(estimate)
        newer=copy.deepcopy(records[-2])
        newer['message']['header']['stamp']['sec']=110
        newer['now_ns']=110_100_000_000
        records.append(newer)
        odom=pose();odom['header']['stamp']['sec']=110
        records.append({'topic':'/odom','message':odom})
        result=c.convert(records,identity,request)
        self.assertTrue(result['complete'],result['missing'])
        self.assertEqual(result['evidence']['scan_stamp_ns'],100_000_000_000)
        self.assertEqual(result['evidence']['odom_stamp_ns'],100_000_000_000)

if __name__=='__main__':unittest.main()
