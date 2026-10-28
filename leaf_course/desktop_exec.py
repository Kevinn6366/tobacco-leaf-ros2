#!/usr/bin/env python3
"""Run a command in the existing Ubuntu desktop session (also over SSH)."""
import os,subprocess,sys
from pathlib import Path
p=Path(__file__).resolve().parent
env=os.environ.copy()
pid=subprocess.check_output(['pgrep','-u',str(os.getuid()),'-x','gnome-shell'],text=True).split()[0]
for e in Path(f'/proc/{pid}/environ').read_bytes().split(b'\0'):
 k,_,v=e.partition(b'=')
 if k.decode() in ('DISPLAY','XAUTHORITY','DBUS_SESSION_BUS_ADDRESS','XDG_RUNTIME_DIR','WAYLAND_DISPLAY'):env[k.decode()]=v.decode()
env['ROS_DOMAIN_ID']='32';env['GZ_PARTITION']='leaf_course';env['QT_X11_NO_MITSHM']='1'
name=sys.argv[1]
commands={'sim':'ros2 launch ./course.launch.py','panel':'python3 app.py','panel-run':'python3 app.py --run','rviz':'rviz2 -d leaf.rviz'}
cmd=commands.get(name,name)
log=open(p/(sys.argv[2] if len(sys.argv)>2 else name+'.log'),'a')
proc=subprocess.Popen(['bash','-c','source /opt/ros/jazzy/setup.bash\nexec '+cmd],env=env,cwd=p,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
(p/(('panel' if 'app.py' in cmd else name.split()[0])+'.pid')).write_text(str(proc.pid));print(proc.pid)
