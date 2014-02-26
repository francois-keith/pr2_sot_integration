#!/usr/bin/python

from math import *

import roslib
roslib.load_manifest('visualization_msgs')
import rospy

from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Pose,Point,Quaternion,Vector3

class MarkerWaveCube:
  def __init__(self, num, dim):
    self.num = num
    self.T = 1.0
    self.alpha = 1.0
    self.namespace = 'stress_test'
    self.size = 0.8

    self._grid = self._make_grid(num, dim)
    self._markers = self._make_marker_grid(self._grid)

    self.wave_funcs_scale = [Wave(0.0, 0.0, [0.0, 0.0, 0.0], 0.5*self.size/num)]*3
    self.wave_funcs_color = [Wave(0.0, 0.0, [0.0, 0.0, 0.0], 0.5)]*4
    self.wave_funcs_color[3].bias = 1.0


  @staticmethod
  def _make_grid(num, dim):
    if dim == 0:
      return [[]]
    else:
      return [ [i / (num-1.0)] + entry for entry in MarkerWaveCube._make_grid(num, dim-1)
                                         for i in range(num) ]


  def _make_marker(self, id, pos):
    """ create a cube marker with position 'pos', sizes 'scale' and
        color 'color'
    """
    pos += [0.0]*(3 - len(pos))
    m = Marker()
    m.id = id
    m.ns = self.namespace
    m.type = Marker.CUBE
    m.header.stamp = rospy.Time.now()
    m.header.frame_id = '/base_link'
    m.pose.position = Point(*pos)
    m.pose.orientation = Quaternion(0, 0, 0, 1)
    m.color.a = self.alpha
    return m

  def _make_marker_grid(self, grid):
    return [self._make_marker(i, pos) for i,pos in enumerate(grid)]
  
  
  def set_wave_scale(self, entry, c, index):
    e = {'x': 0, 'y': 1, 'z': 2}[entry]
    A = (1.0/self.num)*self.size*0.5
    self.wave_funcs_scale[e] = Wave(A, c, index, A)


  def set_wave_color(self, entry, c, index, bias=0.5):
    e = {'r': 0, 'g': 1, 'b': 2, 'a': 3}[entry]
    self.wave_funcs_color[e] = Wave(0.5, c, index, bias)


  def cache_wave(self):
    self._wave = []
    for pos in self._grid:
      w = []
      for f in self.wave_funcs_scale + self.wave_funcs_color:
        w.append(f(pos))
      self._wave.append(tuple(w))


  def apply_modulation(self, t):
    ct = cos(t*self.T)
    st = sin(t*self.T)
    stamp = rospy.Time(t)
    for m,w in zip(self._markers, self._wave):
      m.header.stamp = stamp
      m.scale.x = w[0][0]*ct + w[0][1]*st + w[0][2]
      m.scale.y = w[1][0]*ct + w[1][1]*st + w[1][2]
      m.scale.z = w[2][0]*ct + w[2][1]*st + w[2][2]
      m.color.r = w[3][0]*ct + w[3][1]*st + w[3][2]
      m.color.g = w[4][0]*ct + w[4][1]*st + w[4][2]
      m.color.b = w[5][0]*ct + w[5][1]*st + w[5][2]
      m.color.a = w[6][0]*ct + w[6][1]*st + w[6][2]

class Wave:
  def __init__(self, A, c, index, bias):
    self.A = A
    self.c = c
    self.index = index
    self.bias = bias

  @staticmethod
  def _distance(index, pos):
    """ index = [ix, iy, iz], where every index means:
         1: distance grows in positive direction
        -1: distance grows in negative direction
         0: dimension is ignored
    """
    return sum( ((1-i)/2 + i*p)**2 for i,p in zip(index, pos) )

  def __call__(self, pos):
    re = self.A*cos(self.c*self._distance(self.index, pos))
    im = self.A*sin(self.c*self._distance(self.index, pos))
    return (re, im, self.bias)


pub = rospy.Publisher('visualization_marker', Marker)
rospy.sleep(0.5) # hack to wait for the connections to establish


def test():
  wave_grid = MarkerWaveCube(6,3)

  wave_grid.set_wave_scale('x', 1.5, [1,1,1])

  wave_grid.set_wave_color('r', 1.0, [1,0,0])
  wave_grid.set_wave_color('g', 1.0, [0,1,0])
  wave_grid.set_wave_color('b', 1.0, [0,0,1])

  wave_grid.cache_wave()

  rate = rospy.Rate(10)

  while not rospy.is_shutdown():
    t = rospy.Time.now().to_sec()
    wave_grid.apply_modulation(t)
    for m in wave_grid._markers:
      pub.publish(m)
    rate.sleep()


def main():
  rospy.init_node('test')
  test()


if __name__ == "__main__":
    main()

