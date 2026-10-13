import importlib.util
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

if __name__=='__main__':unittest.main()
