#!/usr/bin/env python

import cv2
import numpy as np
import datetime
import sys

if not (len(sys.argv) > 1 and sys.argv[1] is "debug" ):
        import rospy


# import cv_bridge.CvBridge

from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError


def callback(msg):
    detect_beacon(bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough"))

def detect_beacon(img):
        #convert to HSV encoding - less affected by lighting
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    # set up tools for detecting color
    kernel = np.ones((5,5),np.uint8)
    upper_range = np.array([180,255,255])
    lower_range = np.array([0,100,100])

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
        if w*h < 6000: #TODO change based on video quality
            continue
        centers.append((x+w/2,y+h/2))


        # for debug draw a green rectangle to visualize the bounding rect
        cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)


    # only needed for debug
    cv2.imshow("RGB",image)
    cv2.imshow("Mask",mask)
    cv2.imshow("hsv",hsv)
    cv2.waitKey(2)

    return centers


def stream_webcam():
    camera_port = 0


    #Number of frames to throw away while the camera adjusts to light levels
    ramp_frames = 30

    # Now we can initialize the camera capture object with the cv2.VideoCapture class.
    # All it needs is the index to a camera port.
    camera = cv2.VideoCapture(camera_port)

    # Captures a single image from the camera and returns it in PIL format


    print("Taking image...")
    # Take the actual image we want to keep
     # read is the   easiest way to get a full image out of a VideoCapture object.
    retval, im = camera.read()
    print("back")
    # A nice feature of the imwrite method is that it will automatically choose the
    # correct format based on the file extension you provide. Convenient!
    cv2.imshow("tst", im)

    # You'll want to release the camera, otherwise you won't be able to create a new
    # capture object until your script exits
    del(camera)


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
    print ("here")
    if(len(sys.argv) > 1 and sys.argv[1] == "debug" ):
        stream_webcam()


    bridge = CvBridge()
    vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    cmd_pub = rospy.Publisher("/cmd", String, queue_size=10)
    beacon_found = 0
    #imShowing = False
    #win1 = cv2.imshow("win1",None)
    listener()
    rospy.spin()
