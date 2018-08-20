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


        self.supress_until = datetime.datetime.now()
        self.flag = False
        self.waiting = False

        self.depth_readings = list()



def callback(rgb_msg):
    cur_time = datetime.datetime.now()
    print(cur_time),
    if img_holder.supress_until > cur_time: #or cur_time < img_holder.supress_until:
        print("supressed")
        return
    img_holder.flag = True
    img_holder.rgb_msg = rgb_msg
    #print("rgb w:{}".format(rgb_msg.width))
    #print("rgb h:{}".format(rgb_msg.height))
    img_holder.rgb_img = bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="passthrough")


    img_holder.depth_msg =  rospy.wait_for_message('/camera/depth/image_raw', Image)
    img_holder.depth_img = bridge.imgmsg_to_cv2(img_holder.depth_msg, desired_encoding="passthrough")


    beacon_center = detect_beacon(img_holder.rgb_img)

    # We want the robot to stop for a second before taking a depth reading
    # Can't have him trying to do depth on the fly
    if len(beacon_center) == 0 and img_holder.waiting:
        print("beacon lost")
        cmd_pub.publish("start")
        img_holder.flag = False;
        img_holder.waiting = False
        '''
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = -0.1
        vel_publisher.publish(vel_msg)
        '''

        img_holder.depth_readings = list()
        return



    #print("width", img_holder.rgb_msg.width) # 640
    #print("height", img_holder.rgb_msg.height) # 480
    for c in beacon_center:
            cmd_pub.publish("stop")
            waiting = True

            # create new twiar msg with stop command
            vel_msg = Twist()
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0

            long_wait = 0.2
            short_wait = 0.05

            # attempt to center beacon in image
            if(c[0] > 360): #way too far to the right
                print("Turn right!") #turn left
                clear_invalid_readings()
                vel_msg.angular.z = -0.1
                vel_publisher.publish(vel_msg)
                vel_msg.angular.z = -0.0
                time.sleep(long_wait)
                vel_publisher.publish(vel_msg)


            elif(c[0] > 345): #ittle far to the right
                print("Turn right a lot!") #turn left
                clear_invalid_readings()
                vel_msg.angular.z = -0.1
                vel_publisher.publish(vel_msg)
                time.sleep(short_wait)
                vel_msg.angular.z = -0.0
                vel_publisher.publish(vel_msg)


            elif(c[0] < 305): #little far to the left
                print("Turn left a little!")
                clear_invalid_readings()
                vel_msg.angular.z = +0.1
                vel_publisher.publish(vel_msg)
                time.sleep(short_wait)
                vel_msg.angular.z = -0.0
                vel_publisher.publish(vel_msg)


            elif(c[0] < 290): #too far to the left
                print("Turn left a lot!")
                clear_invalid_readings()
                vel_msg.angular.z = 0.1
                vel_publisher.publish(vel_msg)
                time.sleep(long_wait)
                vel_msg.angular.z = -0.0
                vel_publisher.publish(vel_msg)
            else:
                #beacon more or less in centre of the screen:

                #stop
                print("centered... stop!")
                vel_publisher.publish(vel_msg)


                #verify still in center
                #if cur_time < img_holder.wait_until:

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


                img_holder.depth_readings.append(depthValue)
                print("Depth Value at pixel: {}mm".format(depthValue))



            if len(img_holder.depth_readings) > 5:
                avg_depth = np.average(img_holder.depth_readings)
                print("Avarage Depth Value at pixel: {}mm".format(avg_depth))
                depth_pub.publish(str(avg_depth))
                img_holder.depth_readings = list()
                img_holder.supress_until = cur_time + datetime.timedelta(0,6)
                cmd_pub.publish("start")

            time.sleep(1)
            return


def clear_invalid_readings():
    print("beacon not centered")
    img_holder.depth_readings = list()


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
        cv2.rectangle(hsv, (x, y), (x+w, y+h), (0, 255, 0), 2)


    cv2.imshow("img", image)
    cv2.waitKey(2)

    return centers



if __name__ == '__main__':

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/camera/color/image_raw', Image, callback, queue_size=1)

    # spin() simply keeps python from exiting until this node is stopped
    print("waiting")

    bridge = CvBridge()
    img_holder = image_holder()
    vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10) #TODO change back
    cmd_pub = rospy.Publisher("/cmd", String, queue_size=10)
    depth_pub = rospy.Publisher("/depth_reading", String, queue_size=10)
    beacon_found = 0

    # depth_image = None
    # imShowing = False
    # win1 = cv2.imshow("win1",None)   listener()
    rospy.spin()
