#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=32 GZ_PARTITION=leaf_course
if [[ -f sim.pid ]] && kill -0 "$(cat sim.pid)" 2>/dev/null; then
  echo '课程仿真正在运行。需要重新开始场景时先执行 ./stop.sh。'
  exit 0
fi
python3 build_scene.py
python3 desktop_exec.py sim
python3 desktop_exec.py panel
python3 desktop_exec.py rviz
