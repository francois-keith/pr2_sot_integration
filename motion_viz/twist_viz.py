#!/usr/bin/python

# show a twist in rviz

from math import *

import roslib
roslib.load_manifest('motion_viz')
import rospy
import tf
import PyKDL as kdl
import tf_conversions.posemath as posemath
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Pose,Point,Quaternion,Vector3,Twist
from motion_viz.msg import Jacobian

import marker

colors = [ColorRGBA(0.6, 0.3, 0.3, 1),
          ColorRGBA(0.3, 0.6, 0.3, 1),
          ColorRGBA(0.3, 0.3, 0.6, 1),
          ColorRGBA(0.9, 0.5, 0.5, 1),
          ColorRGBA(0.5, 0.9, 0.5, 1),
          ColorRGBA(0.5, 0.5, 0.9, 1)]


def axis_marker(tw, id = 0, ns = 'twist'):
  """ make a marker message showing the instantaneous
      rotation axis of a twist message"""
  min_length = 0.03

  t = kdl.Twist(kdl.Vector(tw.linear.x,  tw.linear.y,  tw.linear.z),
                kdl.Vector(tw.angular.x, tw.angular.y, tw.angular.z))

  try:
    (x,     rot)  = listener.lookupTransform(target_frame, ref_frame, rospy.Time(0))
    (trans, rotp) = listener.lookupTransform(target_frame, ref_point, rospy.Time(0))
  except (tf.LookupException, tf.ConnectivityException):
    print 'tf exception!'
    return marker.create(id=id, ns=ns, action=Marker.DELETE)

  # put the twist in the right frame
  f = posemath.fromTf( (trans, rot) )
  t = f*t

  direction = t.rot
  location = t.rot * t.vel / kdl.dot(t.rot, t.rot)

  lr = t.rot.Norm()
  lt = t.vel.Norm()

  m = marker.create(id=id, ns=ns, type=Marker.CYLINDER)

  if lr < 0.0001 and lt < 0.0001:
    return marker.create(id=id, ns=ns, action=Marker.DELETE)

  if lr < 0.001:
    # very short axis, assume linear movement
    location = kdl.Vector(0,0,0)
    direction = t.vel
    if lt < min_length:
      direction *= min_length / lt
    m.type = Marker.CUBE
    m = marker.align(m, location, location + direction, 0.02)
  elif lr < min_length:
    direction = direction / lr * min_length
    m = marker.align(m, location - direction, location + direction, 0.02)
  else:
    #BAH! How do I make this better?
    m = marker.align(m, location - direction, location + direction, 0.02)

  m.header.frame_id = target_frame
  m.frame_locked = True

  if(use_colors):
    m.color = colors[id % len(colors)]
  else:
    m.color = ColorRGBA(0.3, 0.3, 0.3, 1)

  return m

# handle a twist
def twist_callback(msg):
  global marker_id, node_name
  pub.publish(axis_marker(msg, marker_id, node_name))

# handle a set of twists
def jac_callback(msg):
  global node_name
  for i,twist in enumerate(msg.columns):
    pub.publish(axis_marker(twist, i, node_name))

# use message to override frames 
def locate(msg, callback, attribute):
  global target_frame, ref_frame, ref_point
  target_frame = msg.header.frame_id
  ref_frame = msg.ref_frame
  ref_point = msg.ref_point
  callback(msg.__getattribute(attribute))


# distribute supported message types
def any_callback(msg):
  if msg._connection_header['type'].endswith('Twist'):
    twist_callback(Twist().deserialize(msg._buff))
  elif msg._connection_header['type'].endswith('Jacobian'):
    jac_callback(Jacobian().deserialize(msg._buff))
  elif msg._connection_header['type'].endswith('TwistLocated'):
    locate(TwistLocated().deserialize(msg._buff), twist_callback, 'twist')
  elif msg._connection_header['type'].endswith('JacobianLocated'):
    locate(JacobianLocated().deserialize(msg._buff), jac_callback, 'jacobian')


### main ###

rospy.init_node('twist_viz')

# retrieve frame names from parameters
target_frame = rospy.get_param('~target_frame', '/base_link')
ref_frame = rospy.get_param('~ref_frame', '/base_link')
ref_point = rospy.get_param('~ref_point', ref_frame)

rospy.loginfo('ref_frame=%s, ref_point=%s, target_frame=%s'
               % (ref_frame, ref_point, target_frame))

use_colors = rospy.get_param('~colors', True)
marker_id = rospy.get_param('marker_id', 1)
node_name = rospy.get_name()[1:]

# set up connections
listener = tf.TransformListener()
pub = rospy.Publisher('/visualization_marker', Marker)
sub = rospy.Subscriber('/twist', rospy.msg.AnyMsg, any_callback)

# run
rospy.spin()
