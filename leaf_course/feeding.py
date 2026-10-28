"""Heap mass planning, wrist feedback, independent box scale, physical box delivery."""
import time,json,traceback,numpy as np
from runtime import P
G=9.8
OBSERVE=[.28,.25,.68]

def plan(c,remaining,density=200.,depth_scale=1.):
 # Only measured visible geometry and learned density enter this planner.
 area=max(c['projected_area_m2'],.001)
 depth_limit=.020 if remaining>25 else .008
 depth=float(np.clip(remaining/(1000*density*area)*depth_scale,.002,depth_limit*depth_scale))
 width=float(np.clip(.045+remaining*.0007,.048,.088))
 footprint=area*min(1.,width/.070)
 return dict(depth_m=depth,opening_m=width,predicted_g=footprint*depth*density*1000,area_m2=footprint,inset_m=float(.014*np.clip(remaining/60.,0.,1.)))

def release_into_box(r,drop):
 # An open finger can still support a folded leaf on its toe. Tip that support
 # upward and away; gravity, not pose attachment or teleportation, releases it.
 r.hand(False,force=5.);r.sleep(.8);initial_sign=None
 for attempt in range(2):
  contacts=r.contact_names()
  sides=[side for side,names in contacts.items() if any('leaf_' in name for name in names)]
  if not sides:return
  if initial_sign is None:
   initial_sign=1. if 'right' in sides else -1.
   drop=list(drop)
   # Rotate above the handle's swept volume; tilting near the rim can strike it.
   drop[2]=max(drop[2],.55);r.move(drop,duration=3,linear=True)
   if len(sides)>1:drop[1]-=.075
   elif 'right' in sides:drop[1]-=.10
   r.move(drop,duration=3)
  sign=initial_sign if attempt==0 else -initial_sign
  r.move(drop,duration=3,roll=sign*np.deg2rad(80.));r.sleep(.8)
 if any('leaf_' in name for names in r.contact_names().values() for name in names):
  raise RuntimeError('张爪与转腕后仍有烟叶接触，停止退离盒口')

def deliver_box(r):
 from scipy.spatial.transform import Rotation
 initial=r.box_xyz();t=r.box_pose.rotation;yaw=float(Rotation.from_quat([t.x,t.y,t.z,t.w]).as_euler('xyz')[2]);x,y=initial[:2];grip_z=float(initial[2]+.18+.0834)
 attempt=dict(initial=initial.tolist(),yaw=yaw);r.results['box_delivery_attempt']=attempt
 r.say('BOX GRASP / 按盒子实际位置夹持提手');r.hand(False);r.move([x,y,.57],yaw=yaw);r.move([x,y,grip_z],yaw=yaw);r.hand(True,6.,stiffness=500.,closed_position=0.)
 contacts=r.contact_names();attempt['contacts']=contacts
 if not all(any('box1::body::handle' in c for c in contacts.get(side,[])) for side in ['left','right']):raise RuntimeError('双指未同时夹住盒子提手，禁止交付')
 before=r.box_xyz();r.move([x,y,.57],duration=5,yaw=yaw);r.sleep(.5);lift=r.box_xyz();attempt.update(before=before.tolist(),lift=lift.tolist())
 if lift[2]-before[2]<.06:raise RuntimeError('盒子未被实际提起')
 r.snapshot('box-lift');r.say('DELIVER / 搬运整盒到交付位');r.move([.30,-.40,.57],duration=5);r.move([.10,-.53,.57],duration=5);r.move([.10,-.53,.009+.18+.0834],duration=5);r.hand(False);r.sleep(1);r.move([.10,-.53,.59]);r.sleep(1)
 final=r.box_xyz();r.snapshot('box-delivered')
 if np.linalg.norm(final[:2]-[.10,-.53])>.045 or abs(final[2]-.009)>.035:raise RuntimeError('盒子未稳定落在交付区域')
 return dict(before=before.tolist(),lift=lift.tolist(),final=final.tolist(),contacts=contacts)

def feed(r,target_g=100.,tolerance_g=3.):
 gate_path=P/'mechanics_gate.json'
 if gate_path.exists() and not json.loads(gate_path.read_text()).get('multi_leaf_passed',False):raise RuntimeError('单片夹取已验证，多片保持尚未通过；当前禁止接入视觉定量批次。可运行 bash test_single_leaf.sh 复测单片。')
 if target_g<=0 or tolerance_g<=0 or tolerance_g>=target_g:raise ValueError('请输入有效目标及小于目标的容差')
 r.stop_requested=False;r.wait_ready()
 if getattr(r,'batch_used',False):raise RuntimeError('本场景已有批次，请先复位清空盒子')
 r.batch_used=True
 r.results={'target_g':float(target_g),'tolerance_g':float(tolerance_g),'cycles':[],'delivered_g':0.,'complete':False,'weight_verified':False,'box_delivered':False,'grasp_model':'contact friction only; adaptive point-cloud bite depth and jaw opening'}
 out=P/'evidence';out.mkdir(exist_ok=True);started=time.monotonic();sim0=r.simtime;density=200.;depth_scale=1.;misses=0;failed_grasps=[]
 r.results['started_sim_s']=sim0
 def save():(out/'batch-result.json').write_text(json.dumps(r.results,indent=2))
 def state(s):r.check_stop();r.say(s);save()
 try:
  r.sleep(3.);r.snapshot('scene-start');r.sleep(1.);r.snapshot('scene-settled')
  state('TARE / 空盒称重与腕部去皮');r.hand(False);r.move([.38,0,.55],sideways=False);r.hand(True);wrist_zero,_=r.sample();r.hand(False);scale_zero,noise=r.scale_sample();r.results.update(scale_tare_N=scale_zero,scale_noise_N=noise,wrist_tare_N=wrist_zero.tolist());gravity_axis=wrist_zero/max(np.linalg.norm(wrist_zero),1e-9) if np.linalg.norm(wrist_zero)>1e-6 else np.array([0.,0.,1.])
  if not (.38*G<scale_zero<.44*G):raise RuntimeError('盒子不在秤上或空盒载荷异常，请复位')
  for cycle in range(16):
   net=r.results['delivered_g'];remaining=target_g-net
   if abs(remaining)<=tolerance_g:break
   removing=remaining<-tolerance_g;region='box' if removing else 'pile'
   row={'index':cycle+1,'start_sim_s':r.simtime,'operation':'remove' if removing else 'add','remaining_g':remaining};r.results['cycles'].append(row)
   state('SCAN / 扫描盒内超量烟叶' if removing else 'SCAN / 扫描堆面与可夹区域');r.hand(False);r.move([.28,-.28 if removing else .25,.68]);r.sleep(.5);cs=r.sense(region)
   cs=[c for c in cs if not c['clipped']]
   if not cs:raise RuntimeError('当前视角无完整可夹区域；复位或调整料堆后重试')
   def score(c):
    penalty=sum(30*np.exp(-np.linalg.norm(np.array(c['grasp'][:2])-xy)**2/.0004) for xy in failed_grasps)
    return (c['height_mm']*c['projected_area_m2']/.006 if abs(remaining)>25 else -c['height_mm']-3000*c['projected_area_m2'])-penalty
   # Prefer a clearly observed long midrib over short vein/edge fragments.
   confident=[c for c in cs if c.get('orientation_line_pixels') is not None and np.linalg.norm(np.array(c['orientation_line_pixels'][:2])-c['orientation_line_pixels'][2:])>=50]
   c=max(confident or cs,key=score);bite=plan(c,abs(remaining),density,depth_scale);g=np.array(c['grasp']);yaw=c['yaw']
   floor=r.box_xyz()[2]+.008 if removing else 0.
   # Keep 4 mm ground clearance for stereo uncertainty and wrist compliance.
   bottom=max(floor+.004,g[2]-bite['depth_m']-(0. if removing else .002));hand_z=bottom+.1084
   bite['depth_m']=max(.001,g[2]-bottom);bite['predicted_g']=bite['area_m2']*bite['depth_m']*density*1000
   row.update(vision=c,plan=bite);r.snapshot(f'pile-{cycle+1}-detect')
   hx=g[0];row.update(hand_z_m=hand_z,hand_x_m=hx,grasp_orientation='top-down contact pinch')
   state('GRASP / 从盒内退料' if removing else 'GRASP / 从柔性叶片中部夹取');r.hand(False,opening=bite['opening_m'])
   r.move([hx,g[1],.47],yaw=yaw);r.move([hx,g[1],hand_z],duration=5,yaw=yaw,linear=True)
   r.hand(True,1.2,stiffness=250.,closed_position=0.);row['contacts']=r.contact_names();r.snapshot(f'pile-{cycle+1}-closed')
   if 'ground_' in str(row['contacts']):raise RuntimeError('夹爪碰到地面，停止并检查点云高度')
   joint_feedback=getattr(r,'state',{})
   jaw_sum=sum(joint_feedback.get(f'panda_finger_joint{i}',0.) for i in (1,2))
   row['jaw_sum_after_close_m']=jaw_sum
   if jaw_sum>.10:
    state('SECURE / 小幅预抬并再次夹紧')
    r.move([hx,g[1],hand_z+.025],duration=2,yaw=yaw,linear=True)
    r.hand(True,2.5,stiffness=350.,closed_position=0.)
    row['jaw_sum_after_reclose_m']=sum(r.state.get(f'panda_finger_joint{i}',0.) for i in (1,2))
    row['reclose_force_N']=2.5;row['secured_contacts']=r.contact_names();r.snapshot(f'pile-{cycle+1}-secured')
    if row['jaw_sum_after_reclose_m']>.10:
     r.move([hx,g[1],hand_z],duration=2,yaw=yaw,linear=True);r.hand(False,force=5.)
     row['outcome']='blocked_closure';failed_grasps.append(g[:2]);depth_scale=max(.3,depth_scale*.6);save();continue
   r.move([hx,g[1],.51],duration=6,yaw=yaw,linear=True);r.sleep(.4);row['evaluation_lifted_leaves']=[n for n,ps in r.truth().items() if min(p['xyz'][2] for p in ps)>.15];r.snapshot(f'pile-{cycle+1}-lift')
   state('WRIST / 固定姿态测量夹取重量');r.move([.38,0,.55],duration=4,sideways=False);w,wn=r.sample();grams=float(max(0.,np.dot(w-wrist_zero,gravity_axis))*1000/G);row.update(wrist_mass_g=grams,wrist_force_N=w.tolist(),wrist_std_N=wn.tolist());r.snapshot(f'pile-{cycle+1}-wrist')
   if grams<3:
    failed_grasps.append(g[:2]);misses+=1;depth_scale=min(3.,depth_scale*1.5);r.hand(False);row['outcome']='empty';save()
    if misses>=3:raise RuntimeError('连续夹空，停止；未伪造补料成功')
    continue
   misses=0
   measured_density=grams/(1000*bite['area_m2']*bite['depth_m']);density=float(np.clip(.5*density+.5*measured_density,100.,3500.));r.results['learned_bulk_density_kg_m3']=density
   if not removing and grams>remaining+tolerance_g:
    state('RETURN / 本夹超出余量，退回料堆并缩小夹取');r.move([hx,g[1],.48],duration=4,yaw=yaw,sideways=False);r.move([hx,g[1],max(.32,g[2]+.12)],yaw=yaw,sideways=False);r.hand(False);r.sleep(.5);depth_scale=max(.15,depth_scale*max(.25,remaining/grams)*.7);row['outcome']='returned_oversize';save();continue
   if removing:
    state('REMOVE / 超差退料到烟叶堆');r.move([.40,.25,.40],sideways=False);r.hand(False);r.sleep(.8)
   else:
    state('BOX 1 / 投入盒子 1');box=r.box_xyz();drop=[float(box[0]),float(box[1]+.075),float(box[2]+.34)];r.move([drop[0],drop[1],.55],duration=4);r.move(drop);release_into_box(r,drop)
   r.move([.30,-.02,.58],sideways=False);state('VERIFY / 盒下独立秤复核');scale,std=r.scale_sample();net=(scale-scale_zero)*1000/G
   row.update(scale_after_g=net,scale_std_N=std,change_g=net-r.results['delivered_g'],outcome='measured',cycle_sim_s=r.simtime-row['start_sim_s']);r.results['delivered_g']=net;r.snapshot(f'pile-{cycle+1}-verified');save()
   if (not removing and row['change_g']<3) or (removing and row['change_g']>-3):raise RuntimeError('盒下秤未检测到预期质量变化，停止')
   if not removing and row['change_g']<grams-3.:raise RuntimeError('投料后盒秤增量小于腕重：疑似未完全释放或途中掉落，停止')
   row['outcome']='removed' if removing else 'deposited';save()
   depth_scale=min(1.,depth_scale*1.1)
  error=r.results['delivered_g']-target_g;r.results['error_g']=error
  if abs(error)>tolerance_g:raise RuntimeError('达到补退料次数上限，当前离散夹取量无法满足容差')
  # Independent second settled scale reading is the gate for whole-box delivery.
  state('FINAL CHECK / 整盒最终复核');scale,std=r.scale_sample();net=(scale-scale_zero)*1000/G;r.results.update(delivered_g=net,error_g=net-target_g,final_scale_std_N=std)
  if abs(net-target_g)>tolerance_g:raise RuntimeError('最终复核超差，禁止交付')
  r.results['weight_verified']=True;save();r.results['box_delivery']=deliver_box(r);r.results.update(box_delivered=True,complete=True);state('COMPLETE / 重量复核合格，整盒已交付')
 except Exception as e:r.results['error']=str(e);r.say('STOPPED / '+str(e));traceback.print_exc()
 finally:
  r.results.update(elapsed_sim_s=r.simtime-sim0,elapsed_wall_s=time.monotonic()-started);save()
  (out/'scale-trace.json').write_text(json.dumps({'columns':['sim_s','force_N'],'tare_N':r.results.get('scale_tare_N'),'samples':getattr(r,'scale_trace',[])}))
