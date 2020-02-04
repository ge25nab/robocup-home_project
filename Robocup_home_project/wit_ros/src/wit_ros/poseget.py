#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
import numpy as np
import os

from openpose_ros_msgs.msg import OpenPoseHumanList
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import String
from wit_ros.srv import ListenAndInterpret

class poseget():
    def __init__(self):
        # subscirber and pub
        self.sub1 = rospy.Subscriber('openpose_ros/human_list',OpenPoseHumanList,self.posecallback,queue_size=1)
        self.sub2 = rospy.Subscriber('/darknet_ros/bounding_boxes',BoundingBoxes,self.objectcallback,queue_size=1)
        self.rate = rospy.Rate(2)
        self.pub1 = rospy.Publisher('object',String,queue_size=10)
        self.w = 0
        self.b = 0
        self.target = ''
        # uncomment to plot the figure
        # self.pointsx = []
        # self.pointsy = []
        

    def posecallback(self,data):
        if data.num_humans > 0:
            # read the left arm and right arm of body coordinates
            # left hands points 5,6,7
            left_hands = {}
            left_hands['x'] = [data.human_list[0].body_key_points_with_prob[i].x for i in [5,6,7]]
            left_hands['y'] = [480-data.human_list[0].body_key_points_with_prob[i].y for i in [5,6,7]]
            # right hands points 2,3,4
            right_hands = {}
            right_hands['x'] = [data.human_list[0].body_key_points_with_prob[i].x for i in [2,3,4]]
            right_hands['y'] = [480-data.human_list[0].body_key_points_with_prob[i].y for i in [2,3,4]]

            ## the two lines below is the body parts points, uncomment them if you want to plot the output
            # self.pointsx = [data.human_list[0].body_key_points_with_prob[i].x for i in range(21)]
            # self.pointsy = [480-data.human_list[0].body_key_points_with_prob[i].y for i in range(21)]

            # do the linear regression to find the line of both arms
            x = np.array([right_hands['x'],np.ones(len(right_hands['x']))]).transpose()
            y = np.array(right_hands['y']).transpose()
            ls_solution = np.linalg.pinv(x).dot(y)
            self.w = ls_solution[0]
            self.b = ls_solution[1]
            print('linear regression finised')


    def objectcallback(self,data):

        ## uncomment to plot the figure 
        # plt.figure()
        # plt.xlim(0, 640)
        # plt.ylim(0, 480)
        distance = []
        for i in range(len(data.bounding_boxes)):
            ## calculate the distance between the detected objects and the line of right arm
            xmin = data.bounding_boxes[i].xmin
            xmax = data.bounding_boxes[i].xmax
            ymin = 480-data.bounding_boxes[i].ymin
            ymax = 480-data.bounding_boxes[i].ymax
            center = [(xmax+xmin)/2,(ymax+ymin)/2]
            distance.append(np.abs(self.w*center[0]-center[1]+self.b)/np.sqrt(np.square(self.w)+1))

            ## uncomment below parts to plot the output
            # plt.plot((xmax+xmin)/2,(ymax+ymin)/2,'r*')
            # plt.plot(np.linspace(xmin,xmax,50),np.linspace(ymax,ymax,50))
            # plt.plot(np.linspace(xmin,xmax,50),np.linspace(ymin,ymin,50))
            # plt.plot(np.linspace(xmax,xmax,50),np.linspace(ymin,ymax,50))
            # plt.plot(np.linspace(xmin,xmin,50),np.linspace(ymin,ymax,50))
            # xs = np.linspace(0,640,50)
            # ys = xs*self.w+self.b 
            # plt.plot(xs,ys)
            # plt.scatter(self.pointsx,self.pointsy,c='r')
            # plt.savefig('/home/athomews1920/Desktop/out.png')
        
        # choose the closest object and double check with operator using wit_ros
        idx = distance.index(min(distance))

        # run the node to let tiago speak
        os.system(" rosrun say_something say.py \"do you mean "+data.bounding_boxes[idx].Class+"\"? ")
        
        ## uncomment to print the object name
        ## print(data.bounding_boxes[idx].Class)

        # run the node to let tiago speak
        get_answer = rospy.ServiceProxy("/wit/listen_interpret", ListenAndInterpret)
        resp1 = get_answer()
        if "ye" in resp1.outcome.text:
            os.system(" rosrun say_something say.py \"OK, I will grab "+data.bounding_boxes[idx].Class+"\". ")
            self.target = data.bounding_boxes[idx].Class
            self.sub2.unregister()
            
        ## uncomment below parts to plot the output
        # plt.figure()
        # plt.xlim(0, 640)
        # plt.ylim(0, 480)
        # xs = np.linspace(0,640,50)
        # ys = xs*self.w+self.b 
        # plt.plot(xs,ys)
        # plt.scatter(self.pointsx,self.pointsy,c='r')
        # plt.savefig('/home/athomews1920/Desktop/out.png')
        rospy.sleep(3)


if __name__ == '__main__':
    rospy.init_node('poseget',anonymous=True)
    rospy.sleep(5)
    get_pose = poseget()
    while not rospy.is_shutdown():
        if len(get_pose.target) is not 0:
            # run the node to let tiago speak
            os.system(" rosrun say_something say.py \" I will head to the table \" ")
            os.system("roslaunch navigation_tiago navigation_tiago.launch &")
            get_pose.sub1.unregister()
            while not rospy.is_shutdown():
                get_pose.pub1.publish(get_pose.target)
                get_pose.rate.sleep()
            
    rospy.spin()


    