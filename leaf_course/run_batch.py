import rclpy
from runtime import Robot
from feeding import feed
rclpy.init();r=Robot()
try:feed(r)
finally:rclpy.shutdown();r.thread.join(3)
