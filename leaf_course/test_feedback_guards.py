"""Control-flow guard tests using scripted sensors; not physical success evidence."""
import importlib.util,sys,types,tempfile,unittest
from pathlib import Path
import numpy as np
class FakeRobot:
 def __init__(self,wrist,scale):self.simtime=0.;self.wrist=iter([0]+wrist);self.scales=iter([0]+scale);self.status='';self.returned=0
 def box_xyz(self):return np.array([.49,-.28,.155])
 def wait_ready(self):pass
 def check_stop(self):pass
 def say(self,s):self.status=s
 def hand(self,*a,**kw):pass
 def move(self,*a,**kw):self.simtime+=3
 def sample(self):return np.array([0,0,next(self.wrist)*9.8/1000]),np.zeros(3)
 def scale_sample(self):return 4.018+next(self.scales)*9.8/1000,0.
 def sleep(self,t):self.simtime+=t
 def sense(self,*a):return [dict(grasp=[.43,.25,.22],yaw=0.,height_mm=20.,projected_area_m2=.006,clipped=False)]
 def snapshot(self,*a):pass
 def contact_names(self):return {}
 def truth(self):return {}
class FeedbackGuards(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();stub=types.ModuleType('runtime');stub.P=Path(self.tmp.name);sys.modules['runtime']=stub
  spec=importlib.util.spec_from_file_location('feeding_under_test',Path(__file__).with_name('feeding.py'));self.f=importlib.util.module_from_spec(spec);spec.loader.exec_module(self.f)
  self.called=False
  def forbidden(r):self.called=True;raise AssertionError('Delivery must remain inhibited')
  self.f.deliver_box=forbidden
 def tearDown(self):self.tmp.cleanup()
 def assert_rejected(self,r):self.assertFalse(self.called);self.assertFalse(r.results['complete']);self.assertFalse(r.results['weight_verified']);self.assertIn('error',r.results)
 def test_three_empty_grasps_cannot_complete(self):
  r=FakeRobot([0,0,0],[]);self.f.feed(r,100,3);self.assert_rejected(r);self.assertEqual(len(r.results['cycles']),3)
 def test_repeated_oversize_cannot_complete(self):
  r=FakeRobot([120]*16,[]);self.f.feed(r,100,3);self.assert_rejected(r);self.assertTrue(all(c['outcome']=='returned_oversize' for c in r.results['cycles']))
 def test_independent_final_drift_blocks_delivery(self):
  r=FakeRobot([100],[100,105]);self.f.feed(r,100,3);self.assert_rejected(r);self.assertAlmostEqual(r.results['delivered_g'],105)
 def test_excess_box_weight_enters_removal(self):
  r=FakeRobot([80,20],[120,100,105]);self.f.feed(r,100,3);self.assert_rejected(r);self.assertEqual(r.results['cycles'][1]['operation'],'remove')
 def test_blocked_jaws_never_enter_full_transport(self):
  r=FakeRobot([],[]);r.state={'panda_finger_joint1':.065,'panda_finger_joint2':.065};moves=[]
  original_move=r.move
  def record(xyz,*args,**kwargs):moves.append(xyz);original_move(xyz,*args,**kwargs)
  r.move=record;self.f.feed(r,100,3);self.assert_rejected(r)
  self.assertTrue(all(c.get('outcome')=='blocked_closure' for c in r.results['cycles']))
  self.assertFalse(any(abs(x[2]-.51)<1e-9 for x in moves))
  self.assertTrue(all('wrist_mass_g' not in c for c in r.results['cycles']))
 def test_no_box_gain_blocks_delivery(self):
  r=FakeRobot([20],[0]);self.f.feed(r,100,3);self.assert_rejected(r)
if __name__=='__main__':unittest.main()
