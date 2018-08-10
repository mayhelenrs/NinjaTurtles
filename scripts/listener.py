#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published
## to the 'chatter' topic

import rospy
import cv2
import numpy as np
import datetime
# import cv_bridge.CvBridge

from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError


def callback(msg):
    #print("got {} by {} image".format(msg.height, msg.width))
    cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
    mask = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
    #get mask for pink
    kernel = np.ones((5,5),np.uint8)
    mask = cv2.inRange(cv_img, np.array([120,40,90]),np.array([255,120,180]))
    mask = cv2.erode(mask,kernel,iterations = 5)
    mask = cv2.dilate(mask,kernel,iterations = 7)
    img, contours, hier = cv2.findContours(mask, cv2.RETR_TREE,\
                                            cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        # get the bounding rect
        x, y, w, h = cv2.boundingRect(c)
        # draw a green rectangle to visualize the bounding rect
        cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)



    win1 = cv2.imshow("win1",image)
    win1 = cv2.imshow("win2",mask)
    cv2.waitKey(2)

    if (np.sum(mask == 255) >14500 and beacon_found is 0):
            print("pink")
            msg = rospy.wait_for_message('/camera/depth/image_raw', JointTrajectoryControllerState)
            #cmd_pub.publish("stop - {}".format(datetime.datetime.now()))
            print("stop - {}".format(datetime.datetime.now()))

'''
    img_hsv = None
    cv2.cvtColor(image,img_hsv,cv2.COLOR_RGB2HSV);
    namedWindow("win1", CV_WINDOW_AUTOSIZE);
    imshow("win1", img_hsv);
    waitkey()
'''


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/camera/color/image_raw', Image, callback)
    # spin() simply keeps python from exiting until this node is stopped
    print("waiting")
if __name__ == '__main__':
    bridge = CvBridge()
    vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    cmd_pub = rospy.Publisher("/cmd", String, queue_size=10)
    beacon_found = 0
    #imShowing = False
    #win1 = cv2.imshow("win1",None)
    listener()
    rospy.spin()
