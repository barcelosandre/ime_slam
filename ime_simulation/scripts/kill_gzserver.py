#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from subprocess import call

if __name__ == '__main__':
 rospy.init_node('kill_gzserver')
 rospy.loginfo(call(["pidof", "gzserver"]))
 #while not rospy.is_shutdown():
 a = call(["pidof", "gzserver"])
 #rospy.loginfo('Killing gzserver...')
 if a == 0:
  rospy.loginfo('Killing gzserver...')
  call(["killall", "gzserver"])
  rospy.loginfo('gzserver kill done!')
  call(["roslaunch", "ime_simulation simulated_hardware.launch"])
  rospy.signal_shutdown('0')
 else:
  call(["roslaunch", "ime_simulation simulated_hardware.launch"])
  rospy.signal_shutdown('0')
 rospy.spin()
