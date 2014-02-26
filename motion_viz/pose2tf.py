#!/usr/bin/python

# moves a tf around according to a twist message

from math import *

import roslib
roslib.load_manifest('motion_viz')
import rospy
import tf
import PyKDL as kdl
import tf_conversions.posemath as pm

from geometry_msgs.msg import Pose

rospy.init_node('pose2tf')

pose = kdl.Frame()

def callback(msg):
  global pose
  pose = pm.fromMsg(msg)

frame_name = rospy.get_param('~frame_name', '/frame')
parent_name = rospy.get_param('~parent_name', '/base_link')
f = rospy.get_param('~rate', 30)

rospy.Subscriber("pose", Pose, callback)
tf_pub = tf.TransformBroadcaster()

rate = rospy.Rate(f)

while not rospy.is_shutdown():
  tf_pub.sendTransform(pose.p, pose.M.GetQuaternion(),
                       rospy.Time.now(), frame_name, parent_name)
  rate.sleep()
