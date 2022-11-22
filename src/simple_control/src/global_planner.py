#!/usr/bin/env python
import rospy
import time
import math
import numpy as np
from threading import Lock
from grid_class import Grid

from geometry_msgs.msg import Vector3, PoseStamped, TwistStamped, Vector3Stamped
from std_msgs.msg import String, Bool, Float64
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge

class GlobalPlanner():

  def __init__(self):
    time.sleep(3)
    self.rate = 10

    self.PLANNING_ROUTE = 0
    self.OPENING_DOOR = 1
    self.MOVING = 2
    self.state = -1

    self.map_width = rospy.get_param("/global_planner/map_width", 23)
    self.map_height = rospy.get_param("/global_planner/map_height", 23)
    self.grid = Grid(self.map_width, self.map_height)

    self.tower_pos = Vector3()
    self.lidar_reading = LaserScan()
    self.drone_pos = Vector3(0, 0, 0)

    # subscribers
    self.tower_pos_sub = rospy.Subscriber("/tower_pos", Vector3, self.tower_pos_callback, queue_size=1)
    self.lidar_sub = rospy.Subscriber("/uav/sensors/lidar", LaserScan, self.lidar_callback, queue_size=1)
    self.drone_pos_sub = rospy.Subscriber("/uav/sensors/gps", Vector3Stamped, self.drone_pos_callback, queue_size=1)
    self.keys_sub = rospy.Subscriber("/keys", String, self.keys_callback, queue_size=1)

    self.MainLoop()

  def tower_pos_callback(self, msg):
    self.tower_pos = msg

  def lidar_callback(self, msg):
    print("lidar sub laserscan:")
    print(msg)
    self.lidar_reading = msg
    self.grid.update(msg, self.drone_pos)
    if self.state == -1 and self.grid.updates == 5:
        self.state = self.PLANNING_ROUTE

  def drone_pos_callback(self, msg):
    self.drone_pos = msg.vector


  # This is the main loop of this class
  def MainLoop(self):

    rate = rospy.Rate(self.rate)

    while not rospy.is_shutdown():
      print("main loop")
      rate.sleep()

  # Called on ROS shutdown
  def shutdown_sequence(self):
    rospy.loginfo(str(rospy.get_name()) + ": Shutting Down")


if __name__ == '__main__':
  rospy.init_node('global_planner')
  try:
    ktp = GlobalPlanner()
  except rospy.ROSInterruptException:
    pass
