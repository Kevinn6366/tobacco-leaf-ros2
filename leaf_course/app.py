#!/usr/bin/env python3
import sys,time,threading,traceback,json,signal,fcntl,subprocess
_lock=open(__file__+".lock","w")
try:fcntl.flock(_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
except BlockingIOError:raise SystemExit("A control panel is already running")
import numpy as np,cv2,rclpy
from python_qt_binding.QtWidgets import QApplication,QWidget,QVBoxLayout,QHBoxLayout,QLabel,QPushButton,QDoubleSpinBox
from python_qt_binding.QtCore import QTimer,Qt
from python_qt_binding.QtGui import QImage,QPixmap
from runtime import Robot,P
from scene_view import SceneView
rclpy.init();r=Robot();view=SceneView(r)
settings=json.loads((P/'settings.json').read_text()) if (P/'settings.json').exists() else {'target_g':100.,'tolerance_g':3.}
app=QApplication([]);signal.signal(signal.SIGINT,lambda *_:app.quit());signal.signal(signal.SIGTERM,lambda *_:app.quit());w=QWidget();w.setWindowTitle('烟叶定量上料 | ROS 2 · 双目 RGB · 接触夹持');w.resize(520,570);w.move(580,45)
w.setStyleSheet('QWidget{background:#15202d;color:#e6edf3;font-size:13px;} QPushButton{background:#236e68;padding:10px;border:0;border-radius:5px;} QPushButton:disabled{background:#394554;}')
lay=QVBoxLayout(w);title=QLabel('眼在手上视觉引导 · 多片烟叶夹取');title.setStyleSheet('font-size:20px;font-weight:bold');lay.addWidget(title)
info=QLabel('地面取料区 → 单盒称重区 → 整盒交付区');lay.addWidget(info)
status=QLabel('等待传感器');status.setWordWrap(True);lay.addWidget(status)
row=QHBoxLayout();mass=QLabel('盒子 1 净重：0.00 g');mass.setStyleSheet('font-size:24px;color:#66ddbc');row.addWidget(mass);target=QDoubleSpinBox();target.setRange(10,200);target.setValue(settings["target_g"]);target.setSuffix(' g 批重');row.addWidget(target);lay.addLayout(row)
live_mass=QLabel('秤实时净重：等待去皮');lay.addWidget(live_mass)
row=QHBoxLayout();row.addWidget(QLabel("允许误差"));tolerance=QDoubleSpinBox();tolerance.setRange(.5,20);tolerance.setValue(settings["tolerance_g"]);tolerance.setSuffix(" g");row.addWidget(tolerance);reset=QPushButton("复位 / 全部归位");row.addWidget(reset);lay.addLayout(row)
row=QHBoxLayout();views=[]
for text in ['左 RGB / 检测','右 RGB','双目计算深度']:
 col=QVBoxLayout();col.addWidget(QLabel(text));v=QLabel();v.setFixedSize(160,120);col.addWidget(v);views.append(v);row.addLayout(col)
lay.addLayout(row);metrics=QLabel('点云等待中');metrics.setWordWrap(True);lay.addWidget(metrics);grasp_info=QLabel('夹取估算等待中');grasp_info.setWordWrap(True);lay.addWidget(grasp_info)
row=QHBoxLayout();run=QPushButton('开始定量上料');inspect=QPushButton('观察 / 点云');stop=QPushButton('停止流程');row.addWidget(run);row.addWidget(inspect);row.addWidget(stop);lay.addLayout(row)
footer=QLabel('双目 RGB → 视差深度 → 点云 /course/points    |    数据保存 evidence/\n柔性为离散弹性近似，材料参数未经实物标定');lay.addWidget(footer)
def worker(fn):
 if r.busy:return
 r.busy=True
 def go():
  try:fn()
  except Exception as e:r.say('ERROR: '+str(e));traceback.print_exc()
  finally:r.busy=False
 threading.Thread(target=go,daemon=True).start()
def observe():r.stop_requested=False;r.wait_ready();r.hand(False);r.move([.28,.22,.68]);r.sleep(.4);r.sense();r.snapshot('manual-inspect');r.say('READY — 点云已更新')
def start():
 from feeding import feed
 goal=target.value();tol=tolerance.value();save_settings();worker(lambda:feed(r,goal,tol))
def save_settings():
 (P/'settings.json').write_text(json.dumps({'target_g':target.value(),'tolerance_g':tolerance.value()}))
def reset_all():
 save_settings();r.stop_requested=True;reset.setEnabled(False);run.setEnabled(False);inspect.setEnabled(False);r.say('RESET / 正在归位，界面将自动重新打开')
 subprocess.Popen([sys.executable,str(P/'reset_scene.py')],cwd=P,stdout=open(P/'reset.log','a'),stderr=subprocess.STDOUT,start_new_session=True)
reset.clicked.connect(reset_all)
run.clicked.connect(start);inspect.clicked.connect(lambda:worker(observe));stop.clicked.connect(lambda:setattr(r,'stop_requested',True))
last=0

def refresh():
 global last
 status.setText(r.status);run.setEnabled(not r.busy and not getattr(r,"batch_used",False));inspect.setEnabled(not r.busy);target.setEnabled(not r.busy);tolerance.setEnabled(not r.busy)
 tare=r.results.get('scale_tare_N')
 if tare is not None:
  live_mass.setText('秤实时净重：'+(f'{(r.scale_load-tare)*1000/9.8:.2f} g' if time.monotonic()-r.scale_last<1 and not r.results.get('weight_verified') else '已结束称重 / 交付阶段'))
 mass.setText('已复核净重：'+format(r.results.get('delivered_g',0),'.2f')+' g')
 if 'rgb' in r.frames:
  rgb=r.annotated if r.annotated is not None and not r.busy else r.frames['rgb'][2];d=r.frames.get('depth')
  im=[rgb,r.frames.get("right",(None,None,None))[2],None]
  if d is not None:im[2]=cv2.cvtColor(cv2.applyColorMap(np.uint8(np.nan_to_num(np.clip((d[2]-.15)/.85,0,1))*255),cv2.COLORMAP_TURBO),cv2.COLOR_BGR2RGB)
  for v,a in zip(views,im):
   if a is not None:
    a=np.ascontiguousarray(cv2.resize(a,(160,120)));v.setPixmap(QPixmap.fromImage(QImage(a.data,160,120,480,QImage.Format_RGB888).copy()))
 metrics.setText(f'双目有效深度 {r.stereo_quality:.0%}  |  可见抓取区域 {len(r.candidates)}  |  点云点数 {getattr(r,"valid_cloud_points",0)}  |  可见高度区域 {len(getattr(r,"height_regions",[]))}  |  仿真时间 {r.simtime:.1f} s')
 rows=r.results.get('cycles',[])
 if rows:
  latest=rows[-1];bite=latest.get('plan',{});grasp_info.setText(f"本夹预计 {bite.get('predicted_g',0):.1f} g  |  腕重 {latest.get('wrist_mass_g',0):.1f} g\n下探 {1000*bite.get('depth_m',0):.1f} mm  |  开口 {1000*bite.get('opening_m',0):.1f} mm")
 if time.monotonic()-last>2:
  (P/'status.json').write_text(json.dumps({'status':r.status,'busy':r.busy,'sim_time_s':r.simtime,'box_xyz':None if r.box_pose is None else [r.box_pose.translation.x,r.box_pose.translation.y,r.box_pose.translation.z],'results':r.results},indent=2));last=time.monotonic()
t=QTimer();t.timeout.connect(refresh);t.start(200);w.show()
if '--run' in sys.argv:QTimer.singleShot(4000,start)
if '--observe' in sys.argv:QTimer.singleShot(1500,lambda:worker(observe))
app.exec_();rclpy.shutdown();r.thread.join(3)
