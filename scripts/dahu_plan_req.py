#!/usr/bin/env python

# Author: christoph.roesmann@tu-dortmund.de

import rospy, math
from geometry_msgs.msg import Polygon
from dahu_msgs.msg import *

def turtleX():
  robot = RobotModel()
  robot.model_id = "turtleX"
  robot.base_radius = 0.25
  robot.max_vel_x = 0.4
  robot.max_vel_x_backwards = 0.2
  robot.max_vel_y = 0.0
  robot.max_vel_theta = 0.3
  robot.acc_lim_x = 0.5
  robot.acc_lim_y = 0.5
  robot.acc_lim_theta = 0.5
  robot.min_turning_radius = 0
  robot.wheelbase = 1.0
  robot.cmd_angle_instead_rotvel = False
  robot.is_footprint_dynamic = False

  robot.min_manipulation_distance = 0.26
  robot.max_manipulation_distance = 0.40
  robot.manipulation_cone_angle = math.pi / 12.0
  return robot

def publish_req_msg():
  pub = rospy.Publisher('/dahu/plan_request', PlanReq, queue_size=1)
  rospy.init_node("test_req_msg")

  # needed so that the first message gets published
  rospy.sleep(0.5)

  req = PlanReq()

  req.robots.append(turtleX())

  a = Area()
  a.name = "Area1"
  a.center.x = 1
  a.center.y = 1
  a.radius = 1
  req.areas.append(a)
  a = Area()
  a.name = "Area2"
  a.center.x = -1
  a.center.y = 1
  a.radius = 0.6
  req.areas.append(a)

  o = Object()
  o.name = "A"
  o.radius = 0.03
  req.objects.append(o)
  o = Object()
  o.name = "B"
  o.radius = 0.05
  req.objects.append(o)

  rp = RobotPose()
  rp.robot_id = "turtleX"
  req.robot_poses.append(rp)

  op = ObjectPose()
  op.object_id = "A"
  req.object_poses.append(op)
  op = ObjectPose()
  op.object_id = "B"
  req.object_poses.append(op)

  oi = ObjectIn()
  oi.area_id = "Area1"
  oi.object_pose_id = 0
  oi.constraint_id = "C0"
  req.constraints.objects_in.append(oi)
  oi = ObjectIn()
  oi.area_id = "Area2"
  oi.object_pose_id = 1
  oi.constraint_id = "C1"
  req.constraints.objects_in.append(oi)

  cm = CanManipulate()
  cm.robot_pose_id = 0
  cm.object_pose_id = 0
  req.constraints.manipulations.append(cm)


  pub.publish(req)

  rospy.sleep(1)




if __name__ == '__main__': 
  try:
    publish_req_msg()
  except rospy.ROSInterruptException:
    pass

