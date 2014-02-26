#!/usr/bin/python

from math import *

import roslib
roslib.load_manifest('visualization_msgs')
import rospy
import PyKDL as kdl

from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Pose,Point,Quaternion,Vector3,Wrench


def color_code(index, intervals=[0.0, 0.5, 1.0],
                      colors=[[0.0, 1.0, 0.0, 1.0],
                              [1.0, 1.0, 0.0, 1.0],
                              [1.0, 0.0, 0.0, 1.0]]):

  # handle corner cases
  if index <= intervals[0] or isnan(index):
    return colors[0]

  if index >= intervals[-1]:
    return colors[-1]

  # pick interval
  (i0, i1) = ((i-1,i+1) for i,t in enumerate(intervals) if t >= index).next()
  (t0, t1) = intervals[i0:i1]
  (c0, c1) = colors[i0:i1]

  t = (index-t0)/(t1-t0)
  return [(1-t)*x0 + t*x1 for (x0,x1) in zip(c0,c1)]


def create(**args):
  m = Marker(**args)
  # make shure the marker is displayable
  if not args.has_key('header'):
    m.header.stamp = rospy.Time.now()
    m.header.frame_id = '/base_link'
  if not args.has_key('color'):
    m.color = ColorRGBA(0.3, 0.3, 0.3, 1)
  if not args.has_key('orientation'):
    m.pose.orientation = Quaternion(0, 0, 0, 1)
  if not args.has_key('scale'):
    m.scale = Vector3(1, 1, 1)
  return m


def align(marker, frm, to, width):
  """ aligns and scales a marker to connect two given points in space
      (works for arrows, cylinders and cubes.

      expected types:
      * marker: visualzation_msgs/Marker
      * frm, to: kdl.Vector
      * width: scalar
  """
  eps = 1e-10 # some small number for alignment test

  direction = to - frm
  scale = direction.Norm()

  if marker.type == Marker.ARROW:
    marker.pose.position = Point(frm.x(), frm.y(), frm.z())
    marker.scale = Vector3(x=0.771*scale, y=width, z=width)

    axis = kdl.Vector(1,0,0) * direction
    angle = direction.x()
  else:
    midpoint = (frm + to)*0.5
    marker.pose.position = Point(midpoint.x(), midpoint.y(), midpoint.z())
    marker.scale = Vector3(x=width, y=width, z=scale)

    axis = kdl.Vector(0,0,1) * direction
    angle = direction.z()

  laxis = axis.Norm()
  if laxis > eps and scale > eps:
    l  = sqrt((1 - angle / scale) / 2) / laxis
    qu = sqrt((1 + angle / scale) / 2)
    q = [axis.x()*l, axis.y()*l, axis.z()*l, qu]
  else:
    # aligned with x/z-axis or zero-length: no rotation needed
    q = [0, 0, 0, 1]

  marker.pose.orientation = Quaternion(*q)
  return marker


def arrow(frm, to, width, ns=''):
  marker = create(ns=ns, type=Marker.ARROW)
  marker = align(marker, frm, to, width)
  return [marker]


def sector(vector1, vector2, base, ns='', id=1):
  """ Create a sector between 'vector1', 'vector2' and 'base'.

  This marker is a triangle list. The interpolation used
  is SLERP and the vectors may be of different lengths.

  """
  marker = create(ns=ns, id=id, type=Marker.TRIANGLE_LIST)

  l_v1 = vector1.Norm()
  l_v2 = vector2.Norm()

  if l_v1 == 0 or l_v2 == 0:
    marker.action = Marker.DELETE
    return marker

  l_arc = acos(kdl.dot(vector1, vector2) / (l_v1 * l_v2))

  if l_arc == 0:
    marker.action = Marker.DELETE
    return marker

  n_steps = int(l_arc / 0.1)
  v_last = vector1 + base
  for i in range(1,n_steps+1):
    t = float(i) / n_steps

    # perform SLERP
    v = (1-t)*vector1 + t*vector2  # interpolate vectors
    l = (1-t)*l_v1 + t*l_v2        # interpolate lengths
    v = v * l / v.Norm()           # set vector length
    v = v + base                   # add origin

    marker.points.append(Point(v_last.x(), v_last.y(), v_last.z()))
    marker.points.append(Point(base.x(), base.y(), base.z()))
    marker.points.append(Point(v.x(), v.y(), v.z()))

    v_last = v

  if marker.points == []:
      marker.action = 2

  return marker

def publish(marker):
  if publish.publisher == None:
    publish.publisher = rospy.Publisher('visualization_marker', Marker)
    rospy.sleep(0.5) # hack to wait for the connections to establish
  publish.publisher.publish(marker)
publish.publisher = None

publish.publisher = None
