import rclpy,time,json,traceback
from runtime import Robot,P
rclpy.init();r=Robot()
try:
 r.wait_ready();r.belt(0);r.say('Moving to inspection');r.move([.28,.22,.68]);time.sleep(3);print('STATE',r.state,flush=True);c=r.sense();print('CANDIDATES',json.dumps(c),flush=True);print('ROOTS',json.dumps({k:v[0]['xyz'] for k,v in r.truth().items()}),flush=True);print('CONTACTS',r.contact_names(),flush=True);r.snapshot('inspection')
except Exception:traceback.print_exc()
finally:rclpy.shutdown()
