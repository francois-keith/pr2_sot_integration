#!/usr/bin/python

# publishes a tf tree

from math import *

import roslib
roslib.load_manifest('tf')
roslib.load_manifest('kdl')

import rospy
import tf
import PyKDL as kdl

from tf.msg import tfMessage
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion


#tf_sender = tf.TransformBroadcaster()

rospy.init_node('tf_tree')
tf_pub = rospy.Publisher('/tf', tfMessage)



def get_tree(frame, depth, step, frame_name, bush=False, idepth=0):
  branch_factor = get_tree.branch_factor
  twig_height   = get_tree.twig_height

  tfs = []
  F = kdl.Frame()

  for i in range(branch_factor):
    F.M = kdl.Rotation.RotZ(2.0*pi*i/branch_factor)
    F.p = F.M*kdl.Vector(step, 0, twig_height) + bush*frame.p

    msg = TransformStamped()
    msg.header.stamp = rospy.Time.now()
    msg.transform.translation = Vector3(*F.p)
    msg.transform.rotation = Quaternion(*(F.M.GetQuaternion()))

    fr_name = frame_name +str(i)
    msg.child_frame_id  = fr_name
    msg.header.frame_id = fr_name[:-(1 + idepth*bush)]

    tfs.append(msg)

    #recurse
    if depth > 1:
      tfs.extend(get_tree(F, depth - 1, step / 2.0,
                          fr_name, bush, idepth + 1))

  return tfs



depth                  = rospy.get_param('depth', 3)
step                   = rospy.get_param('step', 0.4)
get_tree.branch_factor = rospy.get_param('branch_factor', 5)
get_tree.twig_height   = rospy.get_param('twig_height', 0.1)
bush_topo              = rospy.get_param('bush_topoloy', False)
r                      = rospy.get_param('rate', 10)

rate = rospy.Rate(r)

while not rospy.is_shutdown():
  msg = tfMessage()
  msg.transforms = get_tree(kdl.Frame(), depth, step, '/fr', bush_topo)
  tf_pub.publish(msg)
  rate.sleep()
