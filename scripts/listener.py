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
