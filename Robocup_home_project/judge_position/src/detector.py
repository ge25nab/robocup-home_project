#!/usr/bin/env python

from __future__ import division

# Python imports
import numpy as np
import os, sys, cv2, time, math
# from skimage.transform import resize

# ROS imports
import rospy
import std_msgs.msg
from rospkg import RosPack
from std_msgs.msg import UInt8, Bool, String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Polygon, Point32
# from yolov3_pytorch_ros.msg import BoundingBox, BoundingBoxes
from cv_bridge import CvBridge, CvBridgeError

# package = RosPack()
# package_path = package.get_path('yolov3_pytorch_ros')

# Deep learning imports
import torch
import torchvision
import torchvision.transforms as transforms
from torch.utils.data import DataLoader
from torchvision import datasets
from torch.autograd import Variable

from models import *
from utils.utils import *

# Detector manager class for YOLO
class DetectorManager():
    def __init__(self):

        self.weights_path = "/home/athomews1920/ros/workspace/final_robocup/src/judge_position/src/weight/net_param.pkl"

        self.image_topic = '/xtion/rgb/image_raw'

        self.position_topic = "/image_position"

        # self.publish_image = rospy.get_param('~publish_image')
        
        # Initialize width and height
        self.h = 0
        self.w = 0
        


#$#############################################################333

        # Load net
        self.model = resnet_new()
        self.model.load_state_dict(torch.load(self.weights_path))
        # if torch.cuda.is_available():
        #     self.model.cuda()
        # else:
        #     raise IOError('CUDA not found.')
        self.model.eval() # Set in evaluation mode
        rospy.loginfo("Deep neural network loaded")

###############################################################




        # Load CvBridge
        self.bridge = CvBridge()

        # Load classes
        # self.classes = load_classes(self.classes_path) # Extracts class labels from file
        # self.classes_colors = {}
        
        # Define subscribers
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.imageCb, queue_size = 1, buff_size = 2**24)

        # Define publishers
        self.pub_ = rospy.Publisher(self.position_topic, String, queue_size=1)
        self.rate = rospy.Rate(0.5)

        # self.pub_viz_ = rospy.Publisher(self.published_image_topic, Image, queue_size=10)
        rospy.loginfo("Launched node for object detection")

        # Spin
        rospy.spin()


    def imageCb(self, data):
        # Convert the image to OpenCV
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
        except CvBridgeError as e:
            print(e)

        # Configure input
        input_img = self.imagePreProcessing(self.cv_image)
        input_img = Variable(input_img.type(torch.FloatTensor))
        
        # # Get detections from network
        with torch.no_grad():
            detections = self.model(input_img)
        # print(detections)
        # left = detections[0][0].detach()
        # right = detections[0][1].detach()
        # empty = detections[0][2].detach()
        idx,output = torch.max(detections[0],axis=0)
        output = int(output)
        if output is 0:
            output_position = "empty"
        elif output is 1:
            output_position = "left"
        elif output is 2:
            output_position = "right"
        # output_position = True if right>left else False

        # Publish detection results
        self.pub_.publish(output_position)
        self.rate.sleep()

        return True
    

    def imagePreProcessing(self, img):
        # Extract image and shape
        # img = np.copy(img)
        # img = img.astype(float)
        # height, width, channels = img.shape
        
        # if (height != self.h) or (width != self.w):
        #     self.h = height
        #     self.w = width
            
        #     # Determine image to be used
        #     self.padded_image = np.zeros((max(self.h,self.w), max(self.h,self.w), channels)).astype(float)
            
        # # Add padding
        # if (self.w > self.h):
        #     self.padded_image[(self.w-self.h)//2 : self.h + (self.w-self.h)//2, :, :] = img
        # else:
        #     self.padded_image[:, (self.h-self.w)//2 : self.w + (self.h-self.w)//2, :] = img
        
        # # Resize and normalize
        transform = torchvision.transforms.Compose([transforms.ToPILImage(),transforms.Resize((64,48)),transforms.ToTensor()])
        
        input_img = transform(img)
        input_img = input_img.unsqueeze(0)
        # print(input_img.shape)
        # input_img = resize(self.padded_image, (self.network_img_size, self.network_img_size,3))/255.

        # # Channels-first
        # input_img = np.transpose(input_img, (2, 0, 1))

        # # As pytorch tensor
        # input_img = torch.from_numpy(input_img).float()
        # input_img = input_img[None]

        return input_img



if __name__=="__main__":
    # Initialize node
    rospy.init_node("detector_manager_node")

    # Define detector object
    dm = DetectorManager()
