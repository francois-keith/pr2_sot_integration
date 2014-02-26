#!/usr/bin/python

# show a force in rviz

from math import *

import roslib
roslib.load_manifest('motion_viz')
import rospy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Pose,Point,Quaternion,Vector3,Wrench

import PyKDL as kdl
import marker

rospy.init_node('force_viz')

# still a playground.


def axis_areas(frm, to, ns=''):
  m1 = marker.create(id=11, ns=ns, type=Marker.TRIANGLE_LIST)
  m1.color = ColorRGBA(1, 0, 0, 0.3)

  m1.points.append(Point(to[0], to[1], to[2]))
  m1.points.append(Point(to[0], frm[1], frm[2]))
  m1.points.append(Point(frm[0], frm[1], frm[2]))

  m2 = marker.create(id=12, ns=ns, type=Marker.TRIANGLE_LIST)
  m2.color = ColorRGBA(0, 1, 0, 0.3)

  m2.points.append(Point(to[0], to[1], to[2]))
  m2.points.append(Point(frm[0], to[1], frm[2]))
  m2.points.append(Point(frm[0], frm[1], frm[2]))

  m3 = marker.create(id=13, ns=ns, type=Marker.TRIANGLE_LIST)
  m3.color = ColorRGBA(0, 0, 1, 0.3)

  m3.points.append(Point(to[0], to[1], to[2]))
  m3.points.append(Point(frm[0], frm[1], to[2]))
  m3.points.append(Point(frm[0], frm[1], frm[2]))

  return [m1, m2, m3]


def cylinders(frm, to, diameter, ns=''):

  m1 = marker.create(id=21, ns=ns, type=Marker.CYLINDER)
  m1.color = ColorRGBA(1, 0, 0, 0.7)
  m1 = marker.align(m1, frm, kdl.Vector(to[0], frm[1], frm[2]), diameter)

  m2 = marker.create(id=22, ns=ns, type=Marker.CYLINDER)
  m2.color = ColorRGBA(0, 1, 0, 0.7)
  m2 = marker.align(m2, frm, kdl.Vector(frm[0], to[1], frm[2]), diameter)

  m3 = marker.create(id=23, ns=ns, type=Marker.CYLINDER)
  m3.color = ColorRGBA(0, 0, 1, 0.7)
  m3 = marker.align(m3, frm, kdl.Vector(frm[0], frm[1], to[2]), diameter)

  return [m1, m2, m3]


def wrench_markers(w):
  markers = []
  zero = kdl.Vector(0.0, 0.0, 0.0)
  force = kdl.Vector(w.force.x, w.force.y, w.force.z)
  torque = kdl.Vector(w.torque.x, w.torque.y, w.torque.z)

  markers.extend(marker.arrow(zero, force, 0.5, 'force_arrow'))
  markers.extend(axis_areas(zero, force, 'force_areas'))
  markers.extend(cylinders(zero, force, 0.1, 'force_cylinders'))

  markers.extend(marker.arrow(zero, torque, 0.5, 'torque_arrow'))
  markers.extend(axis_areas(zero, torque, 'torque_areas'))
  markers.extend(cylinders(zero, torque, 0.1, 'torque_cylinders'))

  return markers

# main #
pub = rospy.Publisher('/visualization_marker_array', MarkerArray)
rospy.sleep(1)

w = Wrench
w.force = Vector3(1, 0.3, 0.6)
w.torque = Vector3(-0.5, 0.6, 0.3)

mrk = MarkerArray()
mrk.markers.extend(wrench_markers(w))

pub.publish(mrk)

rospy.sleep(1)
