#!/usr/bin/env python2
import os
import rospy
from actionlib import SimpleActionClient
from std_msgs.msg import String
from std_srvs.srv import Empty
from wit_ros.srv import ListenAndInterpret
class interact():
    def __init__(self):

        rospy.loginfo("start to interact with human")

        self.image_topic = '/image_position'

        self.sub1 = rospy.Subscriber(self.image_topic, String, self.get_position,queue_size=1)

        self.pub_left = rospy.Publisher('table', String, queue_size= 100)
        self.rate = rospy.Rate(10)
        

        self.condition = 'empty'
        # rospy.spin()
        

    def get_position(self,data):

        self.condition = data.data

        rospy.loginfo(data.data)

        return True



if __name__ == '__main__':
    rospy.init_node('interaction')
    rospy.sleep(3.0)
    interact_instance = interact()

    for i in range(10):

        if interact_instance.condition == "empty":
            rospy.loginfo("nothing detected")

            # os.system(" rosrun say_something say.py \" i can not recognize you, please try again \" ")
            rospy.sleep(5.0)
            # rospy.Subscriber(interact_instance.image_topic, String, interact_instance.get_position,queue_size=1)
            continue

        if interact_instance.condition == "left":
            os.system(" rosrun say_something say.py \" do you mean the table in your left \" ")
            rospy.sleep(5.0)
            

            get_answer = rospy.ServiceProxy("/wit/listen_interpret", ListenAndInterpret)
            resp1 = get_answer()
            if "ye" in resp1.outcome.text:
                os.system(" rosrun say_something say.py \"ok i will go to left side \" ")

                #publish a rostopic left
                # os.system("roslaunch navigation_tiago navigation_tiago.launch")
                while not rospy.is_shutdown():
                    interact_instance.pub_left.publish("left")
                    interact_instance.rate.sleep()

                break


            elif "no" in resp1.outcome.text:
                os.system(" rosrun say_something say.py \"please try again\" ")
                rospy.sleep(3.0)
                # rospy.Subscriber(interact_instance.image_topic, String, interact_instance.get_position,queue_size=1)

                continue

            else:
                os.system(" rosrun say_something say.py \" i can not understand you please try again \" ")
                rospy.sleep(3.0)
                # rospy.Subscriber(interact_instance.image_topic, String, interact_instance.get_position,queue_size=1)

                continue


        if interact_instance.condition == "right":
            os.system(" rosrun say_something say.py \" do you mean the table in your right \" ")
            rospy.sleep(3.0)


            get_answer = rospy.ServiceProxy("/wit/listen_interpret", ListenAndInterpret)


            resp2 = get_answer()

            rospy.loginfo(resp2.outcome.text)


            if "ye" in resp2.outcome.text:
                os.system(" rosrun say_something say.py \"ok i will go to right side\" ")

                # and then start to grasp things in right side
                
                # os.system("roslaunch navigation_tiago navigation_tiago.launch &")
                while not rospy.is_shutdown():
                    interact_instance.pub_left.publish("right")
                    interact_instance.rate.sleep()

                break

            elif "no" in resp2.outcome.text:
                os.system(" rosrun say_something say.py \" please try again \" ")
                rospy.sleep(3.0)
                # rospy.Subscriber(interact_instance.image_topic, String, interact_instance.get_position,queue_size=1)

                continue

            else:
                os.system(" rosrun say_something say.py \" i can not understand you please try again \" ")
                rospy.sleep(3.0)
                # rospy.Subscriber(interact_instance.image_topic, String, interact_instance.get_position,queue_size=1)

                continue
        

        rospy.spin()