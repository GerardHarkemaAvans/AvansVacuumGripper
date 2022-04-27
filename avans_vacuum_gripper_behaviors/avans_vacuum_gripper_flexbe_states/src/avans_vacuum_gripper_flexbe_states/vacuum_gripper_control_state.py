#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from std_msgs.msg import Bool


class VacuumGripperControlState(EventState):
  '''
  Publishes an empty (std_msgs/Bool) message on a given topic name.

  -- target_time 	float 	Time which needs to settle the vacuum

  >= gripper_vacuum_enable            Enable/Disable vacuum.

  <= done             Done publishing.
  '''

  def __init__(self, gripper_vacuum_enable, target_time = 1.0):
    super(VacuumGripperControlState, self).__init__(outcomes=['done'])
    self._topic = '/Gripper_Pomp'
    self.gripper_vacuum_enable = gripper_vacuum_enable
    self._pub = ProxyPublisher({self._topic: Bool})

    # Store state parameter for later use.
    self._target_time = rospy.Duration(target_time)
    self._start_time = None
		
  def execute(self, userdata):
    # if rospy.Time.now() - self._start_time > self._target_time:
    return 'done' # One of the outcomes declared above.

  def on_enter(self, userdata):
    val = Bool()
    val.data = self.gripper_vacuum_enable
    self._pub.publish(self._topic, val)

    self._start_time = rospy.Time.now()
    time_to_wait = (self._target_time - (rospy.Time.now() - self._start_time)).to_sec()

    if time_to_wait > 0:
      Logger.loginfo('Need to wait for %.1f seconds.' % time_to_wait)
      Logger.loginfo('Grasp/Drop object')


