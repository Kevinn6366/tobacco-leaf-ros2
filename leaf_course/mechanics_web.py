"""Local viewer/controller for the frozen mechanics fixture; no Robot command publisher."""
from pathlib import Path
import os,sys,json,time,threading,subprocess,signal,shutil
from http.server import ThreadingHTTPServer,BaseHTTPRequestHandler
import rclpy,cv2
from rclpy.node import Node
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
P=Path(__file__).resolve().parent/'live_mechanics'
lock=threading.Lock();state={'phase':'尚未启动','busy':False,'sim_s':0,'result':None};procs={};jpg=None
bridge=CvBridge();last_image=0.
def stop(name):
 p=procs.pop(name,None)
 if p and p.poll() is None:
  os.killpg(p.pid,signal.SIGINT)
  try:p.wait(timeout=5)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
def launch(name,args):
 procs[name]=subprocess.Popen(args,cwd=P,env=dict(os.environ,LIBGL_ALWAYS_SOFTWARE='1') if name=='gui' else os.environ.copy(),stdout=open(P/(name+'.log'),'a'),stderr=subprocess.STDOUT,start_new_session=True)
def reset():
 global jpg
 with lock:
  state.update(phase='正在复位',busy=True,result=None)
  for n in ['test','gui','sim']:stop(n)
  ev=P/'evidence'
  if ev.exists() and any(ev.iterdir()):
   dst=P.parent/'evidence'/'live-mechanics'/time.strftime('%Y%m%d-%H%M%S');dst.parent.mkdir(parents=True,exist_ok=True);shutil.copytree(ev,dst)
  if ev.exists():shutil.rmtree(ev)
  ev.mkdir();jpg=None
  launch('sim',['ros2','launch','./course.launch.py'])
  launch('gui',['gz','sim','-g','--render-engine-gui','ogre','--gui-config',str(P/'demo.config')])
  state.update(phase='已复位，等待开始三片演示',busy=False,sim_s=0)
def start():
 with lock:
  if state['busy']:return
  if state['result'] is not None:state['phase']='请先复位，再开始下一轮';return
  state.update(busy=True,phase='等待仿真与控制器就绪')
  launch('test',[sys.executable,'contact_stage.py'])
 def watch():
  p=procs['test'];p.wait()
  with lock:
   if procs.get('test') is not p:return
   f=P/'evidence/result.json'
   result=json.loads(f.read_text()) if f.exists() else {'pass_test':False,'error':'演示已中断，需复位'}
   state.update(busy=False,result=result,phase='完成：三片夹取、搬运、释放通过' if result.get('pass_test') else '本轮未通过，请查看结果并复位')
 threading.Thread(target=watch,daemon=True).start()
def image_cb(m):
 global jpg,last_image
 if time.monotonic()-last_image<.5:return
 last_image=time.monotonic();ok,a=cv2.imencode('.jpg',bridge.imgmsg_to_cv2(m,'bgr8'))
 if ok:jpg=a.tobytes()
rclpy.init();node=Node('mechanics_web_viewer');node.create_subscription(Clock,'/clock',lambda m:state.update(sim_s=m.clock.sec+m.clock.nanosec/1e9),10)
node.create_subscription(String,'/course/status',lambda m:state.update(phase=m.data) if state['busy'] else None,10)
node.create_subscription(Image,'/wrist/stereo/left/image',image_cb,10)
threading.Thread(target=lambda:rclpy.spin(node),daemon=True).start()
HTML='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>三片烟叶 · 验收版演示</title><style>body{margin:36px auto;max-width:1000px;padding:0 24px;background:#121d29;color:#e6edf3;font:18px system-ui}h1{font-size:30px}button{padding:14px 24px;margin:10px 14px 16px 0;border:0;border-radius:8px;background:#279d8c;color:white;font:18px system-ui;cursor:pointer}button:disabled{opacity:.4}section{background:#1d2c3b;border-radius:12px;padding:22px;margin:18px 0}img{width:100%;max-width:640px;border-radius:8px}small{color:#b2c1d0}#phase{font-size:25px;color:#79e3c7}pre{white-space:pre-wrap}</style><h1>三片烟叶 · 验收版演示</h1><small>联动夹指 + 真实托唇 · 固定三片叠放场景 · 视觉定量未启用</small><section><div id="phase">连接中…</div><p>仿真时间：<span id="time">—</span> 秒</p><button id="start" onclick="act('start')">开始三片演示</button><button onclick="act('reset')">复位 / 全部归位</button><p>在 Gazebo 窗口观看机械臂全景。完成后点击“复位”，再点“开始三片演示”可重看。</p></section><section><h2>腕部相机实时画面</h2><img id="camera" alt="等待相机画面"><p>固定坐标对准 → 夹起 → 静止保持 → 搬运 → 释放入盒</p></section><section><h2>本轮结果</h2><pre id="result">尚未运行</pre><small>历史验收：单片 3/3、三片 3/3。这里显示本轮真实结果。</small></section><script>let names={'TARE':'去皮 / 校零','FIXED GRASP':'对准、夹拢三片烟叶','VERTICAL LIFT':'夹起并垂直抬升','STATIONARY HOLD':'静止保持，检查是否滑落','TRANSFER':'搬运','WEIGH HOLD':'搬运后保持、腕部称重','RELEASE':'释放入盒、盒下称重复核'};async function act(a){await fetch('/'+a,{method:'POST',headers:{'X-Leaf-Demo':'1'}});poll()}async function poll(){try{let s=await(await fetch('/state')).json();document.querySelector('#phase').textContent=names[s.phase]||s.phase;document.querySelector('#time').textContent=s.sim_s.toFixed(1);document.querySelector('#start').disabled=s.busy||!!s.result;let r=s.result;document.querySelector('#result').textContent=r?JSON.stringify({结果:r.pass_test?'通过':'未通过',理论质量_g:r.expected_g,腕部称重_g:r.wrist_g,盒内净重_g:r.scale_gain_g,释放后残余_g:r.residual_wrist_g,错误:r.error},null,2):'运行后显示腕部称重、落盒净重及是否通过';if(s.image)document.querySelector('#camera').src='/camera.jpg?t='+Date.now()}catch(e){document.querySelector('#phase').textContent='连接暂时中断'}}setInterval(poll,1000);poll()</script></html>'''
class Handler(BaseHTTPRequestHandler):
 def log_message(self,*a):pass
 def send(self,code,data,kind):
  self.send_response(code);self.send_header('Content-Type',kind);self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data)
 def do_GET(self):
  if self.path=='/':self.send(200,HTML.encode(),'text/html; charset=utf-8')
  elif self.path=='/state':self.send(200,json.dumps(dict(state,image=jpg is not None)).encode(),'application/json')
  elif self.path.startswith('/camera.jpg') and jpg:self.send(200,jpg,'image/jpeg')
  else:self.send(404,b'Not found','text/plain')
 def do_POST(self):
  if self.headers.get('X-Leaf-Demo')!='1':self.send(403,b'Forbidden','text/plain');return
  if self.path not in ['/reset','/start']:self.send(404,b'Not found','text/plain');return
  if self.path=='/reset':threading.Thread(target=reset,daemon=True).start()
  else:start()
  self.send(202,b'{}','application/json')
def shutdown(signum,frame):
 for name in ['test','gui','sim']:stop(name)
 raise SystemExit(0)
signal.signal(signal.SIGTERM,shutdown)
signal.signal(signal.SIGINT,shutdown)
server=ThreadingHTTPServer(('127.0.0.1',8082),Handler)
threading.Thread(target=reset,daemon=True).start()
server.serve_forever()
