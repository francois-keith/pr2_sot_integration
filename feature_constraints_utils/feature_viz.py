#!/usr/bin/python
"""A visualization of geometric features and constraint function over them.

This programm attempts to formalize and visualize feature constraint
functions. Geometric features (line segments, plane segments and points) are
represented as LocatedVector objects, consisting of a position and an orientation.
Constraints are expressed over these features as e.g. distance between two
features, perpendicular projection, and their composition.

These functions include Len, Cos, D, Proj_P, Proj_A. Each of these functions
is modeled as a class, having a method compute() and a method show(), the
latter of which returns a ROS Marker array that visualizes this function and
it's components. Compositions of these functions are defined in the map
constraint_functions.
The display is configured from a ConstraintConfig message, the locations are
computed using a tf listener, using the frame_ids which are taken from the
ConstraintConfig message.

"""


import roslib
roslib.load_manifest('motion_viz')
roslib.load_manifest('robohow_common_msgs')
import PyKDL as kdl
import marker
import rospy
import tf
import tf_conversions.posemath as pm


import threading
from robohow_common_msgs.msg import ConstraintConfig, Constraint, Feature
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA



# some convenience color definitions
grey    = ColorRGBA(0.7, 0.7, 0.7, 0.5)
red     = ColorRGBA(0.7, 0.1, 0.1, 1.0)
yellow  = ColorRGBA(0.7, 0.7, 0.1, 0.6)

main_color = grey
constraint_color = red
feature_color = yellow


# config stores some global parameters for the visualization.
_config_ = {'line_width': 0.02,
            'frame_id': '/base_link',
            'ns': 'features',
            'marker_color': main_color,
            'marker_id': 0}


def msg2vec(v):
  """Convert a geometry_msgs/Vector3 to a KDL Vector."""
  return kdl.Vector(v.x, v.y, v.z)


def msg2feature(msg):
  """Convert a constraint_msgs/Feature to a Feature."""
  pos = msg2vec(msg.position)
  dir = msg2vec(msg.direction)
  return FeatureDisplay(msg.type, pos, dir, msg.frame_id)


def marker_base(**args):
  """Create a marker passing on its args, taking _config_ for default values.

  line_scale -- is a scaling factor for _config_['line_width'],
  line_width -- replaces line_width from _config_
  """
  global _config_

  if args.has_key('line_width'):
    line_width = args.pop('line_width')
  elif args.has_key('line_scale'):
    line_width = args.pop('line_scale') * _config_['line_width']
  else:
    line_width = _config_['line_width']

  m = marker.Marker(id=_config_['marker_id'],
                    ns=_config_['ns'], **args)
  m.header.frame_id = _config_['frame_id']
  m.header.stamp = rospy.Time.now()
  m.scale.z = line_width
  m.lifetime = rospy.Duration(0.4)
  if not args.has_key('color'):
    m.color = _config_['marker_color']

  _config_['marker_id'] += 1

  return m


def marker_line(start, end, **args):
  global _config_
  m = marker_base(type=marker.Marker.CYLINDER, **args)
  marker.align(m, start, end, m.scale.z)
  return m


def marker_point(point, **args):
  global _config_
  m = marker_base(type=marker.Marker.SPHERE, **args)
  m.pose.position = Point(point[0], point[1], point[2])
  m.scale.x = m.scale.z
  m.scale.y = m.scale.z
  return m

def marker_versor(point1, point2, **args):
  global _config_
  m = marker_base(type=marker.Marker.ARROW, **args)
  m.color = yellow
  m.points = [Point(point1[0], point1[1], point1[2]), Point(point2[0], point2[1], point2[2])]
  m.scale.x = 0.025
  m.scale.y = 0.05
  return m

NORMAL=0    # the constraint shall be displayed normally, features are hidden
AS_VECTOR=1 # the constraint (or feature) shall be displayed as a vector
FEATURE=2   # the feature shall be displayed


class LocatedVector:
  """A vector with its point-of-origin

  This class is the basic data type for the constraint function computation.
  It defines some common vector operations for convenience and the methods
  compute() and show(). Often, it's members 'pos' nad 'dir' are used directly.

  """
  def __init__(self, pos, dir):
    self.pos = pos
    self.dir = dir

  def __str__(self):
    p = self.pos
    d = self.dir
    return ('LocatedVector(kdl.Vector(%f, %f, %f), kdl.Vector(%f, %f, %f))'
            % (p.x(), p.y(), p.z(), d.x(), d.y(), d.z()))
            

  def __add__(self, other):
    """Add the second vector to the first vector.

    The resulting vector is located at the first vector's position.

    """
    return LocatedObject(self.pos, self.dir + other.dir)

  def __sub__(self, other):
    """Subtract the second vector from the first vector.

    The resulting vector is located at the first vector's position.

    """
    return LocatedVector(self.pos, self.dir - other.dir)

  def __mul__(self, scalar):
    """Scale the length of the vector.

    The position remains the same.

    """
    return LocatedVector(self.pos, self.dir * scalar)

  def compute(self):
    return self

  def show(self, style=NORMAL):
    return [marker_line(self.pos, self.pos + self.dir)]


class FeatureDisplay:
  """A geometric feature (plane, line or point) with position and direction.

  This class also stores the position and direction 'relative' to
  its parent frame, so 'pos' and 'dir' can be re-computed when
  the frame moves. For computation, the method compute() may
  be called, to have all operations for a LocatedVector available.

  Its method show() has two functions: It can either show the feature itself,
  or it can show itself as a vector, depending on the constraint function
  which calls the show() method. The display of the feature itself
  is called from the ConstraintDisplay class directly.

  """
  def __init__(self, type, pos, dir, frame_id):
    self.type = type
    self.rel_pos = pos
    self.rel_dir = dir
    self.pos = pos
    self.dir = dir
    self.frame_id = frame_id

  def transform(self, frame):
    """(re-)compute the position and direction of the feature.

    Given the 'frame', 'pos' and 'dir' are re-computed using the
    stored 'rel_pos' and 'rel_dir' vectors.

    """
    f_now = kdl.Frame(self.rel_pos)
    f_new = frame*f_now
    self.pos = kdl.Vector(f_new.p)
    self.dir = frame.M*self.rel_dir

  def compute(self):
    return LocatedVector(self.pos, self.dir / 2)

  def show(self, style=NORMAL):
    if style == AS_VECTOR:
      return self.compute().show(AS_VECTOR)
    elif style == FEATURE:
      global _config_
      if self.type == 0: # LINE
        return [marker_line(self.pos - self.dir/2, self.pos + self.dir/2,
                            color=feature_color)]
      elif self.type == 1: #PLANE
        #TODO: handle dir.Norm() == 0
        dir = (self.dir / self.dir.Norm()) * _config_['line_width']/2
        return [marker_line(self.pos - dir, self.pos + dir, color=feature_color, line_width=self.dir.Norm())]
      elif self.type == 2: #POINT
        return [marker_point(self.pos, color=feature_color, line_scale=2)]
      elif self.type == 3: #VERSOR
        return [marker_versor(self.pos, self.pos+self.dir* 0.225, color=yellow)]
    else:
      return []


class Len:
  """Compute the length of a LocatedVector."""

  def __init__(self, vector):
    self.vector = vector

  def compute(self):
    vec = self.vector.compute()
    return vec.dir.Norm()

  def show(self, style=NORMAL):
    vec = self.vector.compute()
    len_marker = marker_line(vec.pos, vec.pos + vec.dir,
                             color=constraint_color, line_scale=1.5)
    return self.vector.show(NORMAL) + [len_marker]


class D:
  """Compute the Distance between the positions of two LocatedVectors."""

  def __init__(self, start, end):
    self.start = start
    self.end = end

  def compute(self):
    return LocatedVector(self.start.pos, self.end.pos - self.start.pos)

  def show(self, style):
    return self.compute().show() + self.start.show(NORMAL) + self.end.show(NORMAL)


class Proj_P:
  """Project a LocatedVector 'vec' onto another LocatedVector 'ref'."""

  def __init__(self, vec, ref):
    self.vec = vec
    self.ref = ref

  def compute(self):
    vec = self.vec.compute()
    ref = self.ref.compute()
    denom = ref.dir.Norm()**2
    if denom == 0:
      res = kdl.Vector()
    else:
      res = ref.dir * (kdl.dot(ref.dir, vec.dir) / denom) - vec.dir
    return LocatedVector(ref.pos, res)

  def show(self, style=NORMAL):
    vec = self.vec.compute()
    ref = self.ref.compute()
    res = self.compute()
    along = res.dir + vec.dir

    markers  = self.vec.show(AS_VECTOR) + res.show() + self.ref.show(AS_VECTOR)
    markers += [marker_line(vec.pos, vec.pos + along)]

    return markers

#TODO: Angle and Cos are identicals, except for the display.
class Angle:
  """Compute the Cosine (normalized dot product) between two LocatedVectors."""

  def __init__(self, vec1, vec2):
    self.vec1 = vec1
    self.vec2 = vec2

  def compute(self):
    vec1 = self.vec1.compute()
    vec2 = self.vec2.compute()
    denom = vec1.Norm() * vec2.Norm()
    if denom == 0:
      return 0
    else:
      return kdl.dot(vec1, vec2) / denom

  def show(self, style=NORMAL):
    global _config_
    w = 3*_config_['line_width']

    vec1 = self.vec1.compute()
    vec2 = self.vec2.compute()
    vec2.pos = vec1.pos

    vec1_l = vec1.dir.Norm()
    vec2_l = vec2.dir.Norm()

    if vec1_l == 0:
      v1 = kdl.Vector()
    else:
      v1 = vec1.dir / vec1_l * w

    if vec2_l == 0:
      v2 = kdl.Vector()
    else:
      v2 = vec2.dir / vec2_l * w

    mrk = marker.sector(v1, v2, vec1.pos)
    mrk.ns = _config_['ns']
    mrk.id = _config_['marker_id']
    mrk.header.frame_id = _config_['frame_id']
    mrk.header.stamp = rospy.Time.now()
    mrk.lifetime = rospy.Duration(0.4)
    _config_['marker_id'] += 1
    mrk.color = constraint_color

    return [mrk]


class Cos:
  """Compute the Cosine (normalized dot product) between two LocatedVectors."""

  def __init__(self, vec1, vec2):
    self.vec1 = vec1
    self.vec2 = vec2

  def compute(self):
    vec1 = self.vec1.compute()
    vec2 = self.vec2.compute()
    denom = vec1.Norm() * vec2.Norm()
    if denom == 0:
      return 0
    else:
      return kdl.dot(vec1, vec2) / denom

  def show(self, style=NORMAL):
    global _config_
    w = 3*_config_['line_width']

    vec1 = self.vec1.compute()
    vec2 = self.vec2.compute()
    vec2.pos = vec1.pos

    vec1_l = vec1.dir.Norm()
    vec2_l = vec2.dir.Norm()

    if vec1_l == 0:
      v1 = kdl.Vector()
    else:
      v1 = vec1.dir / vec1_l * w

    if vec2_l == 0:
      v2 = kdl.Vector()
    else:
      v2 = vec2.dir / vec2_l * w

    mrk = marker.sector(v1, v2, vec1.pos)
    mrk.ns = _config_['ns']
    mrk.id = _config_['marker_id']
    mrk.header.frame_id = _config_['frame_id']
    mrk.header.stamp = rospy.Time.now()
    mrk.lifetime = rospy.Duration(0.4)
    _config_['marker_id'] += 1
    mrk.color = constraint_color

    return [mrk] + self.vec1.show(AS_VECTOR) + vec2.show(AS_VECTOR) + self.vec2.show(AS_VECTOR)


class Proj_A:
  """Project a LocatedVector 'vec' perpendicular to another LocatedVector 'ref'."""

  def __init__(self, vec, ref):
    self.vec = vec
    self.ref = ref

  def compute(self):
    vec = self.vec.compute()
    ref = self.ref.compute()
    denom = ref.dir.Norm()**2
    if denom == 0:
      res = kdl.Vector()
    else:
      res = ref.dir * (kdl.dot(ref.dir, vec.dir) / denom)
    return LocatedVector(vec.pos, res)

  def show(self, style=NORMAL):
    vec = self.vec.compute()
    ref = self.ref.compute()
    res = self.compute()
    perp = res.dir - vec.dir
    dist_marker = marker_line(ref.pos, ref.pos + perp)
    return self.vec.show(AS_VECTOR) + res.show(AS_VECTOR) + self.ref.show(AS_VECTOR) + [dist_marker]



# a map from constraint name to a function (Feature x Feature -> Value)
constraint_functions = {
  'distance':  lambda (f_t, f_w) : Len(Proj_P(D(f_t, f_w), f_w)),
  'height':    lambda (f_t, f_w) : Len(Proj_A(D(f_t, f_w), f_w)),
  'angle':          lambda (f_t, f_w) : Angle(f_t, f_w),
  'perpendicular':  lambda (f_t, f_w) : Cos(f_t, f_w),
  'pointing_at':    lambda (f_t, f_w) : Cos(Proj_P(D(f_t, f_w), f_w),
                                            Proj_P(f_t, f_w)),
  }

#TODO: introduce the function At(f1, f2) which returns
# LocatedVector(f2.pos, f1.dir)
# or: LocatedVector.at(vec2) _and_ Feature.at(f2)
# maybe add LocatedVector.__neg__()

class ConstraintDisplay:
  """Collect features and constraint functions for displaying."""

  def __init__(self, base_frame_id):

    self.mylock = threading.Lock()
    self.tool_features = {}
    self.world_features = {}
    self.constraints = {}
    self.base_frame_id = base_frame_id
    self.listener = tf.TransformListener()

  def set_constraints(self, constraints):
    """Set constraints from Constraint messages,"""
    self.mylock.acquire()
    global constraint_functions
    self.tool_features = {}
    self.world_features = {}
    self.constraints = {}
    for c in constraints:
      self.tool_features[c.tool_feature.name]   = msg2feature(c.tool_feature)
      self.world_features[c.world_feature.name] = msg2feature(c.world_feature)
    for c in constraints:
      #TODO an automatic way to do that.
      functionName = ''
      if c.function == 1:
        functionName = 'angle'
      elif c.function == 2:
        functionName = 'distance'
      elif c.function == 3 and c.command.selec == '111':
        functionName = 'height'
      elif c.function == 3 and c.command.selec == '100':
        functionName = 'height'
      elif c.function == 3 and c.command.selec == '011':
        functionName = 'perpendicular'
      elif c.function == 4:
        functionName = 'pointing_at'
      if functionName != '' :
        f_tool  = self.tool_features[c.tool_feature.name]
        f_world = self.world_features[c.world_feature.name]

        constraint = constraint_functions[functionName]((f_tool, f_world))
        self.constraints[c.name] = constraint
#      else:
#        print "constraint function '%s' not found!" % c.function

    self.mylock.release()

  def transform(self):
    for f in self.tool_features.values() + self.world_features.values():
      try:
        frame = self.listener.lookupTransform(self.base_frame_id,
                                              f.frame_id, rospy.Time(0))
      except:
        continue
      f.transform(pm.fromTf(frame))

  def show(self):
    markers = []
    self.mylock.acquire()
    for name in self.tool_features:
      _config_['ns'] = 'tool_feature_' + name
      markers += self.tool_features[name].show(FEATURE)
    for name in self.world_features:
      _config_['ns'] = 'world_feature_' + name
      markers += self.world_features[name].show(FEATURE)
    for name in self.constraints:
      _config_['ns'] = name
      markers += self.constraints[name].show(NORMAL)
    _config_['ns'] = 'features'
    self.mylock.release()
    return markers



def callback(msg):
  constraint_display.set_constraints(msg.constraints)
  for m in constraint_display.show():
    marker.publish(m)



if __name__ == "__main__":

  rospy.init_node('feature_vis')

  # base_frame is an arbitrary frame in which the markers are displayed
  # the marker locations are defined by the feature frame_id's!
  base_frame_id = rospy.get_param('~base_frame', 'odom')
  rospy.loginfo('base frame: ' + base_frame_id)
  _config_['frame_id'] = base_frame_id
  constraint_display = ConstraintDisplay(base_frame_id)

  sub = rospy.Subscriber('/constraint_config', ConstraintConfig, callback)

  rate = rospy.Rate(10)

  while not rospy.is_shutdown():
    constraint_display.transform()
    _config_['marker_id'] = 0
    for m in constraint_display.show():
      marker.publish(m)
    rate.sleep()

