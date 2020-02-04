#!/usr/bin/env python
## this file is for let tiago speak

import rospy
import actionlib
# from pal_interaction_msgs.msg import TtsText
import pal_interaction_msgs.msg

# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from sensor_msgs.msg import JointState

text = "I am ready to serve you"
lang_id  = "en_GB"

if __name__ == "__main__":
  rospy.init_node("speak",log_level=rospy.INFO)
  rospy.loginfo("Waiting for tts server")
  client = actionlib.SimpleActionClient("/tts", pal_interaction_msgs.msg.TtsAction)
  client.wait_for_server()
  rospy.loginfo("...connected.")

  goal = pal_interaction_msgs.msg.TtsActionGoal()
  goal.goal.rawtext.text = text
  goal.goal.rawtext.lang_id = lang_id
  # goal.text = text
  # goal.lang_id = lang_id

  client.send_goal(goal.goal)
  client.wait_for_result(rospy.Duration(3.0))
  rospy.loginfo("finish speaking")
