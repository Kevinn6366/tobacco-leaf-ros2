from pathlib import Path
import xml.etree.ElementTree as E
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
r=E.parse(Path(__file__).with_name('panda.urdf')).getroot();by_child={j.find('child').get('link'):j for j in r.findall('joint')};chain=[];link='panda_hand'
while link in by_child:
 j=by_child[link];chain.insert(0,j);link=j.find('parent').get('link')
lo=[];hi=[]
for i in range(1,8):
 j=r.find(f"joint[@name='panda_joint{i}']");lim=j.find('limit');lo.append(float(lim.get('lower'))+.03);hi.append(float(lim.get('upper'))-.03)
def fk(q):
 t=np.eye(4)
 for j in chain:
  o=j.find('origin');a=np.eye(4)
  if o is not None:a[:3,3]=np.fromstring(o.get('xyz','0 0 0'),sep=' ');a[:3,:3]=Rotation.from_euler('xyz',np.fromstring(o.get('rpy','0 0 0'),sep=' ')).as_matrix()
  t=t@a
  if j.get('type')=='revolute':
   axis=np.fromstring(j.find('axis').get('xyz'),sep=' ');a=np.eye(4);a[:3,:3]=Rotation.from_rotvec(axis*q[int(j.get('name').replace('panda_joint',''))-1]).as_matrix();t=t@a
 return t
R=np.diag([1.,-1.,-1.])
def ik(xyz,seed,yaw=0.,sideways=False,roll=0.):
 seed=np.clip(np.asarray(seed,dtype=float),np.array(lo)+1e-7,np.array(hi)-1e-7)
 R=Rotation.from_euler("z",yaw).as_matrix()@(np.array([[0.,0.,1.],[1.,0.,0.],[0.,1.,0.]]) if sideways else np.diag([1.,-1.,-1.]))@Rotation.from_euler('x',roll).as_matrix()
 def err(q):
  t=fk(q);return np.r_[4*(t[:3,3]-xyz),Rotation.from_matrix(R@t[:3,:3].T).as_rotvec(),.01*(q-seed)]
 fit=least_squares(err,seed,bounds=(lo,hi),max_nfev=500)
 assert np.linalg.norm(fk(fit.x)[:3,3]-xyz)<.003,(xyz,fit.fun)
 return fit.x
