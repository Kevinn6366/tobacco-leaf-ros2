import rclpy,time,json,traceback,numpy as np
from runtime import Robot,P
rclpy.init();r=Robot();report={}
try:
 r.wait_ready();r.belt(0);r.say('TARE');r.move([.38,0,.55]);r.hand(True);tare,_=r.sample();r.hand(False)
 r.say('DETECT');r.move([.28,.22,.68]);r.sleep(.4);c=r.sense()[0];r.snapshot('before');report['detection']=c;g=c['grasp'];yaw=c['yaw'];r.say('ALIGN '+str(g));r.move([g[0],g[1],.4],yaw=yaw);r.move([g[0],g[1],g[2]+.078],yaw=yaw)
 r.say('CLOSE');r.hand(True,2.0);r.snapshot('closed');report['finger_closed']={k:v for k,v in r.state.items() if 'finger' in k};report['contacts']=r.contact_names();r.say('LIFT');r.move([g[0],g[1],.50],duration=4,yaw=yaw);r.sleep(.5);r.snapshot('lifted');report['lifted_roots']={k:v[0]['xyz'] for k,v in r.truth().items()};print(json.dumps(report),flush=True)
 r.move([.38,0,.55],duration=4);loaded,noise=r.sample();report['mass_g']=float(np.linalg.norm(loaded-tare)*1000/9.81);report['noise']=noise.tolist();r.say('MASS '+str(report['mass_g']))
 r.say('RELEASE');r.move([.44,-.29,.50]);r.move([.44,-.29,.39]);r.hand(False);r.sleep(1);r.snapshot('released');report['final_roots']={k:v[0]['xyz'] for k,v in r.truth().items()};report['completed']=True
except Exception as e:traceback.print_exc();report['error']=str(e)
finally:
 (P/'evidence'/'grasp-test.json').write_text(json.dumps(report,indent=2));r.say('DONE');rclpy.shutdown();r.thread.join(2)
