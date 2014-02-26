#!/usr/bin/python

# send marker messages to visualize a KDL chain

# (specialized for pancake baking for now)

from math import *
import thread

import roslib
roslib.load_manifest('motion_viz')
import rospy

import PyKDL as kdl
from tf_conversions import posemath as pm

import marker

from std_msgs.msg import ColorRGBA, Float64MultiArray
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Pose,PoseStamped,Point,Quaternion,Vector3
from motion_viz.msg import ChainState, ChainInfo, ChainSegment

# Note: we don't use kdl chain because
# 1) it does not store enough information for the "special" chains handled here
# 2) it is too restricted w.r.t. the axes that it can store (only RotX, RotY, ...)
#    there is an implementation in kdl now, but not in the python bindings


# TODO: deal with the constant link offset. For virtual linkages we dont have that now...

class Config:
  def __init__(self):
    self.__doc__ = 'dummy config class'

config = Config()
config.error_length = 0.2
config.error_angle  = pi/6
config.axisWidth = 0.02
config.axisLength = 2.5 * config.axisWidth


# create the spatula chain message

def seg_info(name, type, axis, pose_index=-1):
  types = {'rot': ChainSegment.ROTATIONAL, 'trans': ChainSegment.TRANSLATIONAL}
  return ChainSegment(name=name, type=types[type], pose_index=pose_index,
                      axis=Vector3(*axis))

info = ChainInfo()
info.header.frame_id = '/base_link'
info.name = "virtual linkage"

# cylinder coords
info.segments.append(seg_info('angle',    'rot',   [0,0,1]))
info.segments.append(seg_info('distance', 'trans', [1,0,0]))
info.segments.append(seg_info('height',   'trans', [0,0,1]))
# reverse RPY
info.segments.append(seg_info('roll',     'rot',   [1,0,0], 0))
info.segments.append(seg_info('pitch',    'rot',   [0,1,0], 0))
info.segments.append(seg_info('yaw',      'rot',   [0,0,1], 0))


# forward and inverse kinematics

def _mid_pose(chi):
  mid_pos = kdl.Vector(cos(chi[0])*chi[1], sin(chi[0])*chi[1], chi[2])
  mid_pose = kdl.Frame(kdl.Rotation.Rot(kdl.Vector(0,0,1), chi[0])*kdl.Rotation.RotX(pi/2)*kdl.Rotation.RotY(-pi/2), mid_pos)
  return mid_pose

def fk(chi):
  mid_pose = _mid_pose(chi)
  rpy = kdl.Frame(kdl.Rotation.RPY(chi[3], chi[4], chi[5]))
  return [mid_pose * rpy.Inverse()]

def ik(pose):
  angle = atan2(pose.p.y(), pose.p.x())
  distance = sqrt(pose.p.x()**2, pose.p.y()**2)
  height = pose.p.z()

  mid_pose = _mid_pose([angle, distance, height])
  rot = pose * (mid_pose.Inverse()*pose).M.Inverse()

  (roll, pitch, yaw) = rot.GetRPY()

  return [angle, distance, height, roll, pitch, yaw]



def color_code(index, intervals=[0.0, 0.5, 1.0],
                      colors=[[0.0, 1.0, 0.0, 1.0],
                              [1.0, 1.0, 0.0, 1.0],
                              [1.0, 0.0, 0.0, 1.0]]):

  # handle corner cases
  if index <= intervals[0] or isnan(index):
    return ColorRGBA(*colors[0])

  if index >= intervals[-1]:
    return ColorRGBA(*colors[-1])

  # pick interval
  (i0, i1) = ((i-1,i+1) for i,t in enumerate(intervals) if t >= index).next()
  (t0, t1) = intervals[i0:i1]
  (c0, c1) = colors[i0:i1]

  t = (index-t0)/(t1-t0)
  return ColorRGBA(*[(1-t)*x0 + t*x1 for (x0,x1) in zip(c0,c1)])




#factory function for creating Segment objects
def createSegment(segment_info, seg_id, jnt_id, frame_id):
  if segment_info.type == ChainSegment.ROTATIONAL:
    return SegmentRotational(segment_info, seg_id, jnt_id, frame_id)
  if segment_info.type == ChainSegment.TRANSLATIONAL:
    return SegmentTranslational(segment_info, seg_id, jnt_id, frame_id)


class SegmentBase:
  @staticmethod
  def create(segment_info, seg_id, jnt_id, frame_id):
    '''factory function for creating Segment objects'''
    if segment_info.type == ChainSegment.ROTATIONAL:
      return SegmentRotational(segment_info, seg_id, jnt_id, frame_id)
    if segment_info.type == ChainSegment.TRANSLATIONAL:
      return SegmentTranslational(segment_info, seg_id, jnt_id, frame_id)


  def __init__(self, segment_info, seg_id, jnt_id, frame_id):
    self.chi = 0
    self.desired_chi = 0
    self.seg_id = seg_id
    self.jnt_id = jnt_id
    self.set_info(segment_info)

    self.marker = marker.create(ns='chain', id=seg_id)
    self.marker.header.frame_id = frame_id


  def set_info(self, segment_info):
    '''set type and axis of the segment'''
    a = segment_info.axis
    self.name = segment_info.name
    self.axis = kdl.Vector(a.x, a.y, a.z)
    self.pose_base_index = segment_info.pose_index


  def set_state(self, chain_state, chain_pose, base_pose):
    '''set the current pose for the segment'''
    self.chi = chain_state.chi[self.jnt_id]
    self.chi_desired = chain_state.chi_desired[self.jnt_id]
    self.weight = chain_state.weights[self.jnt_id]

    if self.pose_base_index < 0:
      self.pose_base = chain_pose
      return chain_pose * self.pose_segment()
    else:
      self.pose_base = base_pose*chain_state.poses[self.pose_base_index]
      return self.pose_segment()


  def pose_segment(self):
    '''return the relative pose, caused directly by this segment'''
    return kdl.Frame()




class SegmentRotational(SegmentBase):
  def __init__(self, segment_info, seg_id, jnt_id, frame_id):
    SegmentBase.__init__(self, segment_info, seg_id, jnt_id, frame_id)
    self.marker.type = Marker.CYLINDER


  def pose_segment(self):
    '''return the relative pose, caused directly by this segment'''
    return kdl.Frame(kdl.Rotation.Rot2(self.axis, self.chi))


  def update(self):
    '''update the marker to reflect the current joint angle'''

    p1 = self.pose_base * (-self.axis * config.axisLength)
    p2 = self.pose_base * ( self.axis * config.axisLength)

    marker.align(self.marker, p1, p2, config.axisWidth)
    self.marker.color = color_code(abs(self.chi - self.chi_desired) / config.error_angle)
    if self.weight == 0.0:
      self.marker.color = ColorRGBA(0.0, 0.0, 1.0, 1.0)


class SegmentTranslational(SegmentBase):
  def __init__(self, segment_info, seg_id, jnt_id, frame_id):
    SegmentBase.__init__(self, segment_info, seg_id, jnt_id, frame_id)
    self.marker.type = Marker.CUBE


  def pose_segment(self):
    '''return the relative pose, caused directly by this segment'''
    return kdl.Frame(self.axis * self.chi)


  def update(self):
    '''update the marker to reflect the current joint angle'''

    p1 = self.pose_base.p
    p2 = self.pose_base * (self.axis*self.chi)

    marker.align(self.marker, p1, p2, config.axisWidth)
    self.marker.color = color_code(abs(self.chi - self.chi_desired) / config.error_length)
    if self.weight == 0.0:
      self.marker.color = ColorRGBA(0.0, 0.0, 1.0, 1.0)




class ChainDrawer:

  def __init__(self, chain_info=None):
    self.lock=thread.allocate_lock()
    self.pose = kdl.Frame()
    if chain_info:
      self.set_info(chain_info)

  def set_info(self, chain_info):
    with self.lock:
      frame_id = chain_info.header.frame_id
      segment_index = 0
      joint_index = 0
      self.segments = []
      for seg in chain_info.segments:
        self.segments.append(SegmentBase.create(seg, segment_index, joint_index, frame_id))
        if seg.type != ChainSegment.FIXED:
          joint_index += 1
        segment_index += 1

  def set_state(self, chain_state):
    #TODO: fill angles with zeros if there are not enough numbers
    with self.lock:
      chain_pose = self.pose
      for seg in self.segments:
        chain_pose = seg.set_state(chain_state, chain_pose, self.pose)
        seg.update()

  def set_pose(self, pose):
    with self.lock:
      self.pose = pm.fromMsg(pose)

  def get_markers(self):
    with self.lock:
      return [s.marker for s in self.segments]




# main #

rospy.init_node('chain_viz')

global redraw_flag
redraw_flag = True

drawer = ChainDrawer()
drawer.set_info(info)


state = ChainState()
state.chi = [0.0]*6
state.chi_desired = [0.0]*6
state.weights = [1.0]*6
state.poses = [kdl.Frame()]
drawer.set_state(state)

def callback_pose(msg):
  drawer.set_pose(msg)
  redraw()

def callback_end_pose(msg):
  global state
  state.poses = [pm.fromMsg(msg.pose)]
  drawer.set_state(state)


def callback_chi(msg):
  global state
  state.chi = msg.data
  drawer.set_state(state)
  redraw()

def callback_chi_desired(msg):
  global state
  state.chi_desired = msg.data
  drawer.set_state(state)
  redraw()

def callback_weights(msg):
  global state
  state.weights = msg.data
  drawer.set_state(state)
  redraw()

def redraw():
  global redraw_flag
  if redraw_flag:
    redraw_flag = False
    mrks = drawer.get_markers()
    for m in mrks:
      m.header.stamp = rospy.Time.now()
      pub.publish(m)



pub = rospy.Publisher('/visualization_marker', Marker)

sub_chi = rospy.Subscriber("/chi_f", Float64MultiArray, callback_chi)
sub_chi_desired = rospy.Subscriber("/chi_f_desired", Float64MultiArray, callback_chi_desired)
sub_weights = rospy.Subscriber("/weights", Float64MultiArray, callback_weights)
sub_pose = rospy.Subscriber("/object_pose", Pose, callback_pose)
sub_end_pose = rospy.Subscriber("/o1o2_pose", PoseStamped, callback_end_pose)

rate = rospy.Rate(8)


while not rospy.is_shutdown():
  redraw_flag = True
  rate.sleep()

