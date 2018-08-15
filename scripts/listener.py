#!/usr/bin/env python

import cv2
import numpy as np
import datetime
import sys
import rospy


# import cv_bridge.CvBridge

from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError


class image_holder:
    def __init__(self):
        self.rgb_msg = None
        self.depth_msg = None

        self.rgb_img = None
        self.depth_img = None


def callback(rgb_msg):

    img_holder.rgb_msg = rgb_msg
    #print("rgb w:{}".format(rgb_msg.width))
    #print("rgb h:{}".format(rgb_msg.height))
    img_holder.rgb_img = bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="passthrough")
    beacon_center = detect_beacon(img_holder.rgb_img)

    for c in beacon_center:

            pos_x = int(float(c[0])/float(img_holder.rgb_msg.width) * float(img_holder.depth_msg.width))
            pos_y = int(float(c[1])/float(img_holder.rgb_msg.height) * img_holder.depth_msg.height)
            print("Depth coords: {}, {}".format(pos_x,pos_y))
            cv2.rectangle(img_holder.depth_img, (pos_x-3, pos_y-3), (pos_x+3, pos_y+3), (0, 255, 0), 2)
            cv2.imshow("depth", np.multiply(img_holder.depth_img,1.5))
            print("Depth {}".format((img_holder.depth_msg.data[400])))


def callback2(depth_msg):
    img_holder.depth_img = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
    #print(depth_msg.data)
    #cv2.imshow("depth", depth_image)
    #print("d w:{}".format(depth_msg.width))
    #print("d h:{}".format(depth_msg.height))
    img_holder.depth_msg = depth_msg



def detect_beacon(img):

        #convert to HSV encoding - less affected by lighting
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    # set up tools for detecting color
    kernel = np.ones((5,5),np.uint8)
    upper_range = np.array([170,255,255])
    lower_range = np.array([90,100,100])

    # apply blur, mask and erode /dilate to find all pink reigons
    mask = cv2.blur(hsv, (3,3))
    mask = cv2.inRange(mask, lower_range, upper_range) #TODO work out HSV for pink
    mask = cv2.erode(mask,kernel,iterations = 8)
    mask = cv2.dilate(mask,kernel,iterations = 8)
    img2, contours, hier = cv2.findContours(mask, cv2.RETR_TREE,\
                                            cv2.CHAIN_APPROX_SIMPLE)

    centers = list()

    # loop through all the pink edges we've found
    for c in contours:
        # get the bounding rect
        x, y, w, h = cv2.boundingRect(c)

        # add all the large areas to results - filter out anu remaining noise
        #if w*h < 6000: #TODO change based on video quality

        centers.append((x+w/2,y+h/2))

        # for debug draw a green rectangle to visualize the bounding rect
        cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)

    # only needed for debugnp
    cv2.imshow("RGB",image)
    cv2.imshow("Mask",mask)
    cv2.imshow("hsv",hsv)


    cv2.waitKey(2)
    return centers



def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/camera/color/image_raw', Image, callback)
    rospy.Subscriber('/camera/depth/image_raw', Image, callback2)
    # spin() simply keeps python from exiting until this node is stopped
    print("waiting")


if __name__ == '__main__':

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/camera/color/image_raw', Image, callback)
    rospy.Subscriber('/camera/depth/image_raw', Image, callback2)
    # spin() simply keeps python from exiting until this node is stopped
    print("waiting")

    bridge = CvBridge()
    img_holder = image_holder()
    vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    cmd_pub = rospy.Publisher("/cmd", String, queue_size=10)
    beacon_found = 0

    # depth_image = None
    # imShowing = False
    # win1 = cv2.imshow("win1",None)   listener()
    rospy.spin()
