#!/usr/bin/python

# moves a tf around according to a twist message

from math import *

import roslib
roslib.load_manifest('motion_viz')
import rospy
import tf
import PyKDL as kdl
import tf_conversions.posemath as posemath

from geometry_msgs.msg import Twist

rospy.init_node('moveable_frame')

frame = kdl.Frame()
twist = kdl.Twist()

def callback(msg):
  global twist
  lin = kdl.Vector(msg.linear.x, msg.linear.y, msg.linear.z)
  rot = kdl.Vector(msg.angular.x, msg.angular.y, msg.angular.z)
  twist = kdl.Twist(lin, rot)

frame_name = rospy.get_param('~frame_name', '/frame')
parent_name = rospy.get_param('~parent_name', '/base_link')
f = rospy.get_param('~rate', 30)

rospy.Subscriber("twist", Twist, callback)
tf_pub = tf.TransformBroadcaster()

rate = rospy.Rate(f)

while not rospy.is_shutdown():
  t = kdl.Twist(twist)
  #frame = kdl.addDelta(frame, t, 1.0/f) # global mode
  frame = kdl.addDelta(frame, kdl.Frame(frame.M)*t, 1.0/f) # local mode
  tf_pub.sendTransform(frame.p, frame.M.GetQuaternion(),
                       rospy.Time.now(), frame_name, parent_name)
  rate.sleep()
