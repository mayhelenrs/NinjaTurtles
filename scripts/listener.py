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
        self.depth_img_list = list()


        self.supress_until = datetime.datetime.now()
        self.flag = False
        self.waiting = False

        self.depth_readings = list()



def depth_callback(depth_msg):
    #print("Got depth msg")
    cv_img = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
    timestamp=  float(depth_msg.header.stamp.secs) + (float(depth_msg.header.stamp.nsecs)/1000000000.0)
    img_holder.depth_img_list.append((cv_img,timestamp))
    if(len(img_holder.depth_img_list) > 200):
        img_holder.depth_img_list.pop(0)




def rgb_callback(rgb_msg):

    cur_time = datetime.datetime.now()
    #print(cur_time)
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
        print("{} Beacon detected ".format(len(beacon_center))),
        img_holder.depth_img = find_closest_img(img_holder.rgb_msg.header.stamp)

        #turn_to_beacon(c)

            #verify still in center
            #if cur_time < img_holder.wait_until:

        pos_x = int(float(c[0])/float(img_holder.rgb_msg.width) * len(img_holder.depth_img[0]))
        pos_y = int(float(c[1])/float(img_holder.rgb_msg.height) * len(img_holder.depth_img))
        #print("Color coord: {}, {}".format(c[0],c[1]))
        #print("Depth coords: {}, {}".format(pos_x,pos_y))
        cv2.rectangle(img_holder.depth_img, (pos_x-3, pos_y-3), (pos_x+3, pos_y+3), (0, 255, 0), 2)
        #cv2.imshow("depth", np.multiply(img_holder.depth_img,1.5))
        #img_holder.depth_img[y][x] -> I know its confusing... y = column, x = row
        depthValue = 0
        depthCount = 0


        for i in range(max(pos_y-20,0), min(pos_y+20,img_holder.depth_msg.height)): # Iterate through pixel height
            for j in range(max(pos_x-10,0), min(pos_x+10,img_holder.depth_msg.height)): # Iterate through pixel width
                newVal = img_holder.depth_img[i][j]
                if (newVal != 0 and newVal < 1500): # Ignore 0 values
                    depthValue += img_holder.depth_img[i][j]
                    depthCount += 1
        if (depthCount != 0): # We do not want to divide by zero
            depthValue = depthValue / depthCount # Calculate the average

        if (depthValue == 0 or depthValue > 2000):
            print("Value discarded")
            return



        img_holder.depth_readings.append(depthValue)
        #s("Depth Value at pixel: {}mm".format(depthValue))

        time = float(img_holder.rgb_msg.header.stamp.secs) + (float(img_holder.rgb_msg.header.stamp.nsecs)/1000000000.0)

        depth_pub.publish(str(depthValue)+" "+str(c[3])+" "+str(time))

    if len(img_holder.depth_readings) > 1: # How many buffer images
        avg_depth = np.average(img_holder.depth_readings)
        print("Avarage Depth Value at pixel: {}mm".format(avg_depth))
        #depth_pub.publish(str(avg_depth)+" "+str(c[3]))
        img_holder.depth_readings = list()
        img_holder.supress_until = cur_time + datetime.timedelta(0,0)

    return

def find_closest_img(stamp):
    #print("Looking for closest to {}".format(stamp))
    time = float(stamp.secs) + (float(stamp.nsecs)/1000000000.0)
    minDif = None
    toReturn = None
    for t in img_holder.depth_img_list:
        temp = time - t[1]
        #print("Dif:{} - {} = {}".format(time, t[1], temp))
        if (minDif is None) or (abs(temp) < minDif):
            minDif = abs(temp)
            toReturn = t[0]


    #print("MinDif = {}".format(minDif))
    return toReturn




def clear_invalid_readings():
    print("beacon not centered")
    img_holder.depth_readings = list()


def mask_detection(hsv, kernel, mask, image, low_threshold, high_threshold):
    mask = cv2.inRange(mask, low_threshold, high_threshold) #TODO work out HSV for pink
    mask = cv2.erode(mask,kernel,iterations = 8)
    mask = cv2.dilate(mask,kernel,iterations = 8)
    img2, contours, hier = cv2.findContours(mask, cv2.RETR_TREE,\
                                            cv2.CHAIN_APPROX_SIMPLE)
    sorted(contours, key=lambda c: cv2.contourArea(c))
    if len(contours) > 0 and cv2.contourArea(contours[-1]) > 200:
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
    if pink is not None:
        yellow = mask_detection(hsv, kernel, mask, image, yellow_low, yellow_high)
        blue = mask_detection(hsv, kernel, mask, image, blue_low, blue_high)
        green = mask_detection(hsv, kernel, mask, image, green_low, green_high)

        other_color = None
        color_name = ""
        if yellow is not None:
            other_color = yellow
            color_name = "yellow"
        elif blue is not None:
            other_color = blue
            color_name = "blue"
        elif green is not None:
            other_color = green
            color_name = "green"

        other_cY = -1
        if other_color is not None:
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
            beacon_id = -1
            m = cv2.moments(pink)
            cX = int(m["m10"] / m["m00"])
            cY = int(m["m01"] / m["m00"])
            if other_cY != -1 and other_cY < cY:
                if color_name == "blue":
                    beacon_id = 1
                elif color_name == "yellow":
                    beacon_id = 3


            elif other_cY != -1 and cY < other_cY:
                if color_name == "green":
                    beacon_id = 0
                elif color_name == "yellow":
                    beacon_id = 2

            centers.append((cX, cY,(other_cX, other_cY),beacon_id))
            cv2.drawContours(image, [pink], -1, (0, 255, 0), 2)
            cv2.circle(image, (cX, cY), 7, (255, 255, 255), -1)

        # for debug draw a green rectangle to visualize the bounding rect

    # only needed for debugnp
    #cv2.imshow("RGB",image)
    #cv2.imshow("Mask",mask)
    #cv2.imshow("hsv",hsv)

    cv2.waitKey(2)
    return centers

def turn_to_beacon(c):
        # create new twiar msg with stop command
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        short_wait = 0.1
        long_wait = 0.5

        center = img_holder.rgb_msg.width / 2
        wide_margin = 0.2*center
        slim_margin = 0.1*center

        #print("c[0]")
        # attempt to center beacon in image
        if(c[0] > center + wide_margin): #way too far to the right
            print("Turn right!") #turn left
            clear_invalid_readings()
            vel_msg.angular.z = -0.1
            vel_publisher.publish(vel_msg)
            vel_msg.angular.z = -0.0
            time.sleep(long_wait)
            vel_publisher.publish(vel_msg)


        elif(c[0] > center + slim_margin): #ittle far to the right
            print("Turn right a lot!") #turn left
            clear_invalid_readings()
            vel_msg.angular.z = -0.1
            vel_publisher.publish(vel_msg)
            time.sleep(short_wait)
            vel_msg.angular.z = -0.0
            vel_publisher.publish(vel_msg)


        elif(c[0] < center - slim_margin): #little far to the left
            print("Turn left a little!")
            clear_invalid_readings()
            vel_msg.angular.z = 0.1
            vel_publisher.publish(vel_msg)
            time.sleep(short_wait)
            vel_msg.angular.z = -0.0
            vel_publisher.publish(vel_msg)


        elif(c[0] < center - wide_margin): #too far to the left
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




if __name__ == '__main__':

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/camera/color/image_raw', Image, rgb_callback)
    rospy.Subscriber('/camera/depth/image_raw', Image, depth_callback)


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
