#!/usr/bin/env python

import cv2
import numpy as np
import datetime
import sys
import rospy
import time


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

        self.next_update = None
        self.flag = False


def callback(rgb_msg):
    if img_holder.flag:
        return

    img_holder.flag = True

    curtime = datetime.datetime.now()
    img_holder.rgb_msg = rgb_msg
    #print("rgb w:{}".format(rgb_msg.width))
    #print("rgb h:{}".format(rgb_msg.height))
    img_holder.rgb_img = bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="passthrough")
    beacon_center = detect_beacon(img_holder.rgb_img)

    #print("width", img_holder.rgb_msg.width) # 640
    #print("height", img_holder.rgb_msg.height) # 480
    for c in beacon_center:

            # create new twiar msg with stop command
            vel_msg = Twist()
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0

            # attempt to center beacon in image
            if(c[0] > 360): #way too far to the right
                print("Turn right!") #turn left

                vel_msg.angular.z = -0.1
                vel_publisher.publish(vel_msg)
                vel_msg.angular.z = -0.0
                time.sleep(0.4)
                vel_publisher.publish(vel_msg)


            elif(c[0] > 330): #ittle far to the right
                print("Turn right a lot!") #turn left
                vel_msg.angular.z = -0.1
                vel_publisher.publish(vel_msg)
                time.sleep(0.1)
                vel_msg.angular.z = -0.0
                vel_publisher.publish(vel_msg)


            elif(c[0] < 310): #little far to the left
                print("Turn left a little!")
                vel_msg.angular.z = +0.1
                vel_publisher.publish(vel_msg)
                time.sleep(0.1)
                vel_msg.angular.z = -0.0
                vel_publisher.publish(vel_msg)


            elif(c[0] < 300): #too far to the left
                print("Turn left a lot!")
                vel_msg.angular.z = 0.1
                vel_publisher.publish(vel_msg)
                time.sleep(0.4)
                vel_msg.angular.z = -0.0
                vel_publisher.publish(vel_msg)
            else:
                #beacon more or less in centre of the screen:

                #stop
                print("centered... stop!")
                vel_publisher.publish(vel_msg)


                #verify still in center

                #begin process of finding depth

                pos_x = int(float(c[0])/float(img_holder.rgb_msg.width) * len(img_holder.depth_img[0]))
                pos_y = int(float(c[1])/float(img_holder.rgb_msg.height) * len(img_holder.depth_img))
                print("Color coord: {}, {}".format(c[0],c[1]))
                print("Depth coords: {}, {}".format(pos_x,pos_y))
                cv2.rectangle(img_holder.depth_img, (pos_x-3, pos_y-3), (pos_x+3, pos_y+3), (0, 255, 0), 2)
                #cv2.imshow("depth", np.multiply(img_holder.depth_img,1.5))
                #img_holder.depth_img[y][x] -> I know its confusing... y = column, x = row
                depthValue = 0
                depthCount = 0


                for i in range(max(pos_y-10,0), min(pos_y+10,img_holder.depth_msg.height)): # Iterate through pixel height
                    for j in range(max(pos_x-30,0), min(pos_x+30,img_holder.depth_msg.height)): # Iterate through pixel width
                        if (img_holder.depth_img[i][j] != 0): # Ignore 0 values
                            depthValue += img_holder.depth_img[i][j]
                            depthCount += 1
                if (depthCount != 0): # We do not want to divide by zero
                    depthValue = depthValue / depthCount # Calculate the average
                print("Average Depth Value at pixel: {}mm".format(depthValue))

            #time.sleep(1)
            img_holder.flag = False;
            return


def callback2(depth_msg):
    img_holder.depth_img = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
    #print("width:", len(img_holder.depth_img[0])) # 480 -> width
    #print("height:", len(img_holder.depth_img)) # 360 -> height

    #UNCOMMENT FOR DEBUGGING PURPOSES
    #print("NewStart")
    #xTemp = 60;
    #yTemp = 234;
    #for i in range(yTemp-10, yTemp+10):
        #for j in range(xTemp-30, xTemp+30):
            #if img_holder.depth_img[i][j] > 0:
                #print(img_holder.depth_img[i][j], i, j)

    #print(depth_msg.data)
    #cv2.imshow("depth", depth_image)
    #print("d w:{}".format(depth_msg.width))
    #print("d h:{}".format(depth_msg.height))
    img_holder.depth_msg = depth_msg


def mask_detection(hsv, kernel, mask, image, low_threshold, high_threshold):
    mask = cv2.inRange(mask, low_threshold, high_threshold) #TODO work out HSV for pink
    mask = cv2.erode(mask,kernel,iterations = 8)
    mask = cv2.dilate(mask,kernel,iterations = 8)
    img2, contours, hier = cv2.findContours(mask, cv2.RETR_TREE,\
                                            cv2.CHAIN_APPROX_SIMPLE)
    print(len(contours))
    sorted(contours, key=lambda c: cv2.contourArea(c))
    if len(contours) > 0 and cv2.contourArea(contours[-1]) > 6000:
        return contours[-1]
    return None

def detect_beacon(img):

        #convert to HSV encoding - less affected by lighting
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    # set up tools for detecting color
    kernel = np.ones((5,5),np.uint8)

    pink_upper = np.array([170,255,255])
    pink_low = np.array([110,100,100])

    blue_high = np.array([90,255,255])
    blue_low = np.array([20,100,100])

    yellow_high = np.array([100,255,255])
    yellow_low = np.array([40,100,100])

    green_high = np.array([85,255,255])
    green_low = np.array([5,100,39])

    centers = list()
    mask = cv2.blur(hsv, (6,6))

    pink = mask_detection(hsv, kernel, mask, image, pink_low, pink_upper)


    # loop through all the pink edges we've found
    if pink != None:
        yellow = mask_detection(hsv, kernel, mask, image, yellow_low, yellow_high)
        blue = mask_detection(hsv, kernel, mask, image, blue_low, blue_high)
        green = mask_detection(hsv, kernel, mask, image, green_low, green_high)

        other_color = None
        color_name = ""
        if yellow != None:
            other_color = yellow
            color_name = "yellow"
        elif blue != None:
            other_color = blue
            color_name = "blue"
        elif green != None:
            other_color = green
            color_name = "green"

        other_cY = -1
        if other_color != None:
            other_m = cv2.moments(other_color)
            other_cX = int(other_m["m10"] / other_m["m00"])
            other_cY = int(other_m["m01"] / other_m["m00"])
            cv2.drawContours(image, [other_color], -1, (0, 255, 0), 2)
            cv2.circle(image, (other_cX, other_cY), 7, (255, 255, 255), -1)

        # get the bounding rect
        x, y, w, h = cv2.boundingRect(pink)
        # add all the large areas to results - filter out anu remaining noise
        # can use cv2.contourArea(c) to get area.
        #if w*h < 6000: #TODO change based on video quality
        #centers.append((x+w/2,y+h/2, cv2.contourArea(pink)))

        m = cv2.moments(pink)
        cX = int(m["m10"] / m["m00"])
        cY = int(m["m01"] / m["m00"])
        if other_cY != -1 and other_cY > cY:
            centers.append((color_name, "above"))
            centers.append(("pink", "below"))
        elif other_cY != -1 and cY > other_cY:
            centers.append((color_name, "below"))
            centers.append(("pink", "above"))
        cv2.drawContours(image, [pink], -1, (0, 255, 0), 2)
        cv2.circle(image, (cX, cY), 7, (255, 255, 255), -1)

        # for debug draw a green rectangle to visualize the bounding rect



    # only needed for debugnp
    cv2.imshow("RGB",image)
    cv2.imshow("Mask",mask)
    cv2.imshow("hsv",hsv)

    cv2.waitKey(2)


    return centers





'''
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
'''

if __name__ == '__main__':

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/camera/color/image_raw', Image, callback, queue_size=1)
    rospy.Subscriber('/camera/depth/image_raw', Image, callback2, queue_size=1)
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
