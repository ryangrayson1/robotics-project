#!/usr/bin/env python
import rospy
import tf2_ros
import time
import math
import numpy as np
from threading import Lock
from grid_class import Grid
from astar_class import AStar

from geometry_msgs.msg import Vector3, PoseStamped, TwistStamped, Vector3Stamped, PointStamped, Point
from std_msgs.msg import String, Bool, Float64, Int32
from sensor_msgs.msg import LaserScan
from tf2_geometry_msgs import do_transform_point

from environment_controller.srv import use_key, use_keyResponse, use_keyRequest

class GlobalPlanner():

  def __init__(self):
    time.sleep(3)
    self.rate = 10

    self.LOCATING_DOG = 0
    self.PLANNING_ROUTE = 1
    self.OPENING_DOOR = 2
    self.MOVING = 3
    self.state = 0

    self.keys_left = 4
    self.map_width = rospy.get_param("/global_planner/map_width", 23)
    self.map_height = rospy.get_param("/global_planner/map_height", 23)
    self.grid = Grid(self.map_width, self.map_height)
    self.grid_lock = Lock()
    self.astar = AStar(self.grid)

    self.tower_pos = Vector3()
    self.lidar_reading = LaserScan()
    self.drone_pose = PoseStamped()
    self.dog_pos = Vector3()

    # subscribers
    self.tower_pos_sub = rospy.Subscriber("/cell_tower/position", Vector3, self.tower_pos_callback, queue_size=1)
    self.lidar_sub = rospy.Subscriber("/uav/sensors/lidar", LaserScan, self.lidar_callback, queue_size=1)
    self.drone_pose_sub = rospy.Subscriber("/uav/sensors/gps", PoseStamped, self.drone_pose_callback, queue_size=1)
    self.keys_sub = rospy.Subscriber("/keys_remaining", Int32, self.keys_callback, queue_size=1)

    self.position_pub = rospy.Publisher("/uav/input/position", Vector3, queue_size=1)

    # self.use_key_service = rospy.Service('use_key', use_key, self.use_key_function)

    self.tfBuffer = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.tfBuffer)

    self.MainLoop()

  def tower_pos_callback(self, msg):
    if self.state == self.LOCATING_DOG:
      try:
        self.tower_pos = msg
        transform = self.tfBuffer.lookup_transform('cell_tower', 'world', rospy.Time())
        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = 'world'
        point.point.x = msg.x
        point.point.y = msg.y
        point.point.z = msg.z
        new_point = do_transform_point(point, transform)

        self.dog_pos = Vector3(new_point.point.x, new_point.point.y, new_point.point.z)
        dog_x, dog_y = self.grid.world_to_grid(self.dog_pos)
        self.grid.set_cell(dog_x, dog_y, -3)
        print("dog pos:", self.dog_pos)
        self.state = self.PLANNING_ROUTE
      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print('tf2 exception, continuing')

  def lidar_callback(self, msg):
    self.lidar_reading = msg
    self.grid_lock.acquire()
    self.grid.update(self.drone_pose, msg)
    self.grid_lock.release()

    if self.grid.updates == 20:
      self.grid.print_grid()
      rospy.signal_shutdown("test")

  def drone_pose_callback(self, msg):
    # print("drone pos callback")
    self.drone_pose = msg.pose
  
  def keys_callback(self, msg):
    self.keys_left = msg.data

  def plan_route(self):
    # here assume we are at the center of a square, and that we have sufficient lidar data for occupancy grid
    # run A* with occupancy grid, get back just the next step

    # check if next step is a door, if so change state to opening door
    # else publish to position topic to take the step, set state to moving
    self.grid_lock.acquire()
    try:
      self.grid.print_grid()
      grid_x, grid_y = self.astar.get_next_move(self.drone_pose.position, self.dog_pos)
      world_x, world_y = self.grid.grid_to_world(grid_x, grid_y)
      self.next_move = Point(world_x, world_y, 3.0)
      print("next move:", self.next_move)
    except:
      self.grid.print_grid()
      rospy.signal_shutdown("test")
    self.grid_lock.release()

    if self.grid.is_closed_door(self.next_move.x, self.next_move.y):
      self.state = self.OPENING_DOOR
    else:
      self.state = self.MOVING

  def open_door(self):
    rospy.wait_for_service('use_key')
    use_key_function = rospy.ServiceProxy('use_key', use_key)
    res = use_key_function(self.next_move)
    if res.success:
      print("door opened")
      self.grid.set_cell(self.next_move.x, self.next_move.y, -2)
      self.state = self.MOVING
    else:
      print("door not opened")
      self.grid.set_cell(self.next_move.x, self.next_move.y, 101)
      self.state = self.PLANNING_ROUTE

  def move(self):
    self.position_pub.publish(Vector3(self.next_move.x, self.next_move.y, 3.0))
    while abs(self.drone_pose.position.x - self.next_move.x) > .2 or abs(self.drone_pose.position.y - self.next_move.y) > .2:
      time.sleep(.1)
    pass

  # This is the main loop of this class
  def MainLoop(self):

    rate = rospy.Rate(self.rate)

    # DOOR CHECKPOINT - use w = 11, h = 11, seed = 6
    # print("initial keys: " + str(self.keys_left))
    # self.next_move = Point(0, 1, 3.0) # a door
    # self.open_door()
    # time.sleep(1)
    # print("keys after use_key call 1: " + str(self.keys_left))
    # self.next_move = Point(1, 0, 3.0) # not a door
    # self.open_door()
    # time.sleep(1)
    # print("keys after use_key call 2: " + str(self.keys_left))
    # return

    while not rospy.is_shutdown():
      # print("main loop")

      # always updating grid with callback - both for occupancy grid and for detecting doors

      if self.state == self.LOCATING_DOG:
        continue

      if self.state == self.PLANNING_ROUTE:
        self.plan_route()

      elif self.state == self.OPENING_DOOR:
        self.open_door()
        
      elif self.state == self.MOVING:
        # maybe add some abort functionality here if close to a wall
        pass


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
