#!/usr/bin/python

# This program reads a transform from tf and republishes it
# as a PoseStamped message.

import sys
import roslib
roslib.load_manifest('motion_viz')
import rospy
import tf
import tf_conversions.posemath as pm
from geometry_msgs.msg import PoseStamped,Pose, Point,Quaternion


class TF2PoseBase:
  def __init__(self, target_frame, base_frame, topic):
    self.target_frame = target_frame
    self.base_frame = base_frame
    self.listener = tf.TransformListener()

  def wait_for_ready(self, time=10.0):
    try:
      self.listener.waitForTransform(self.base_frame, self.target_frame, rospy.Time(), rospy.Duration(time))
      return True
    except:
      return False

  def pose(self):
    try:
      transform = self.listener.lookupTransform(self.base_frame, self.target_frame, rospy.Time(0))
      return Pose(Point(*transform[0]), Quaternion(*transform[1]))
    except:
      return None

  def pose_kdl(self):
    return pm.fromMsg(self.pose())


class TF2Pose(TF2PoseBase):
  def __init__(self, target_frame, base_frame, topic=None):
    TF2PoseBase.__init__(self, target_frame, base_frame, topic)
    if topic:
      self.pub = rospy.Publisher(topic, Pose)

  def spin(self):
    pose = self.pose() 
    if pose == None:
      return
    self.pub.publish(pose)


class TF2PoseStamped(TF2PoseBase):
  def __init__(self, target_frame, base_frame, topic):
    TF2PoseBase.__init__(self, target_frame, base_frame, topic)
    self.pub = rospy.Publisher(topic, PoseStamped)

  def spin(self):
    p = PoseStamped()
    p.header.stamp = rospy.Time.now()
    p.header.frame_id = self.base_frame
    p.pose = self.pose()
    if p.pose == None:
      return
    self.pub.publish(p)



def main():
  if len(sys.argv) >= 4:
    target_frame = sys.argv[1]
    base_frame = sys.argv[2]
    topic = sys.argv[3]
    if len(sys.argv) >= 5:
      stamped = (sys.argv[4] == '-stamped')
    else:
      stamped = True
  else:
    print 'usage: %s target_frame base_frame topic_name [-stamped|-plain]' % sys.argv[0]
    sys.exit()

  rospy.init_node('tf2pose')
  rate = rospy.Rate(10)

  if stamped:
    posepub = TF2PoseStamped(target_frame, base_frame, topic)
  else:
    posepub = TF2Pose(target_frame, base_frame, topic)

  posepub.wait_for_ready()

  while not rospy.is_shutdown():
    posepub.spin()
    rate.sleep()


if __name__ == "__main__":
    main()

