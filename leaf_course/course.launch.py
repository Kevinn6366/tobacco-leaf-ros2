from pathlib import Path
import json,os
import xml.etree.ElementTree as ET
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
 p=Path(__file__).parent
 engine=os.environ.get('LEAF_PHYSICS_ENGINE')
 if not engine:
  uses_lamina=ET.parse(p/'world.sdf').find(".//collision[@name='lamina_contact']/geometry/mesh") is not None
  engine=str(p/'physics/contact-build/libleaf-contact-physics.so') if uses_lamina else 'gz-physics-bullet-featherstone-plugin'
  if uses_lamina and not Path(engine).is_file():raise RuntimeError('Convex lamina world requires the project physics plugin. Run bash physics/build_contact.sh first.')
 args=['/model/ur/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V','/panda/wrist/loadcell@sensor_msgs/msg/JointState[gz.msgs.Model','/scale/joints@sensor_msgs/msg/JointState[gz.msgs.Model','/world/leaf_course/model/scale/link/tray/sensor/scale_contacts/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts','/scale/wrench@geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench','/panda/wrist/wrench@geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench','/wrist/stereo/left/image@sensor_msgs/msg/Image[gz.msgs.Image','/wrist/stereo/right/image@sensor_msgs/msg/Image[gz.msgs.Image','/wrist/stereo/left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo','/wrist/stereo/right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo','/model/conveyor/link/body/track_cmd_vel@std_msgs/msg/Float64]gz.msgs.Double','/model/conveyor/link/body/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry']
 args += [f'/model/{name}/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V' for name in list(json.loads((p/'scene_manifest.json').read_text()))+['box1']]
 args += [f'/world/leaf_course/model/ur/link/panda_{side}finger/sensor/{side}_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts' for side in ['left','right']]
 return LaunchDescription([
 IncludeLaunchDescription(PythonLaunchDescriptionSource(get_package_share_directory('ros_gz_sim')+'/launch/gz_sim.launch.py'),launch_arguments={'gz_args':'-s -r -v 2 --physics-engine '+engine+' '+str(p/'world.sdf')}.items()),
 Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':(p/'panda.urdf').read_text(),'use_sim_time':True}]),
 Node(package='ros_gz_sim',executable='create',arguments=['-file',str(p/'panda.sdf'),'-name','ur']),
 Node(package='controller_manager',executable='spawner',arguments=['joint_state_broadcaster','panda_arm_controller','--controller-manager-timeout','120']),
 Node(package='ros_gz_bridge',executable='parameter_bridge',arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']),
 Node(package='controller_manager',executable='spawner',arguments=['panda_hand_controller','--controller-manager-timeout','120']),
 Node(package='ros_gz_bridge',executable='parameter_bridge',arguments=args)])
