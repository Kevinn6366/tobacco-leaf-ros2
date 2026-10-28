"""Known disparity and textureless-image safety checks, independent of Gazebo truth."""
import unittest,numpy as np,cv2
from stereo import StereoDepth
class StereoTest(unittest.TestCase):
 def setUp(self):self.s=StereoDepth(dict(width=640,height=480,fx=554.256,fy=554.256,cx=319.5,cy=239.5,baseline_m=.06))
 def test_known_shift_metric_depth(self):
  rng=np.random.default_rng(23);left=rng.integers(20,235,(480,640,3),dtype=np.uint8);left=cv2.GaussianBlur(left,(3,3),0);right=np.zeros_like(left);right[:,:-60]=left[:,60:]
  _,_,z,_,good=self.s.compute(left,right);roi=z[30:-30,160:-150];self.assertGreater(np.isfinite(roi).mean(),.9);self.assertLess(abs(np.nanmedian(roi)-554.256*.06/60),.001)
 def test_blank_images_cannot_create_valid_depth(self):
  blank=np.full((480,640,3),90,np.uint8);_,_,z,_,good=self.s.compute(blank,blank);self.assertFalse(good.any());self.assertTrue(np.isnan(z).all())
if __name__=='__main__':unittest.main()
