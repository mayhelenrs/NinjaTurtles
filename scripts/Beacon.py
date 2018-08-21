#!/usr/bin/env python
import datetime
import sys
import time
import math
from nav_msgs.msg import Odometry, Path

if not (len(sys.argv) > 1 and sys.argv[1] is "debug" ):
        import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped, Pose
from cv_bridge import CvBridge, CvBridgeError
import tf

class curr_position:
    def __init__(self):
        self.beacons_found = None
        self.px = None
        self.py = None
        self.pz = None
        self.ox = None
        self.oy = None
        self.oz = None
        self.ow = None
        self.timestamp = None

def save_robot_pose(currPose):
    if (curr_position.beacons_found[0]+curr_position.beacons_found[1]+curr_position.beacons_found[2]+curr_position.beacons_found[3] >= 4):
        cmd_pub.publish("go_home")
        print(curr_position.beacons_found)  
    curr_position.timestamp = datetime.datetime.now()
    curr_position.px = currPose.position.x
    curr_position.py = currPose.position.y
    curr_position.pz = currPose.position.z
    curr_position.ox = currPose.orientation.x
    curr_position.oy = currPose.orientation.y
    curr_position.oz = currPose.orientation.z
    curr_position.ow = currPose.orientation.w
    position_list.append(curr_position)
    if (len(position_list) >= 1000):
        position_list.pop(0)
    return

def callback(msg):
    #quaternion = (0,0,1,0)
    closest_time = -1
    bRobot_position = curr_position
    for i in position_list:
        if (i.timestamp < closest_time) or (i.timestamp < 0):
            bRobot_position = i
            closest_time = i.timestamp
    quaternion = (bRobot_position.ox, bRobot_position.oy, bRobot_position.oz, bRobot_position.ow)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    print("roll", euler[0])
    print("pitch", euler[1])
    print("yaw", euler[2])
    msgSplit = msg.data.split()
    print(msgSplit)
    depth = (float(msgSplit[0])/1000) - 0.15
    offset = 0.050
    length = math.sqrt((depth*depth) + (offset*offset))
    theta = math.atan(offset / depth)
    x = (math.cos(euler[2]-theta) * length) + bRobot_position.px
    y = (math.sin(euler[2]-theta) * length) + bRobot_position.py

    #x = (math.cos(euler[2]) * depth) + curr_position.px
    #y = (math.sin(euler[2]) * depth) + curr_position.py
    marker = Marker();
    marker.header.frame_id = "map";
    marker.id = int(msgSplit[1])   
    marker.header.stamp = rospy.Time.now()
    marker.type = Marker.CYLINDER
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0.1
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1
    marker.scale.x = 0.15
    marker.scale.y = 0.15
    marker.scale.z = 0.25
    marker.color.a = 1.0
    if(int(msgSplit[1]) == 0): # Green
        curr_position.beacons_found[0] = 1
        marker.color.r = 0
        marker.color.g = 255
        marker.color.b = 0       
    elif(int(msgSplit[1]) == 1): # Blue
        curr_position.beacons_found[1] = 1
        marker.color.r = 0
        marker.color.g = 0
        marker.color.b = 255        
    elif(int(msgSplit[1]) == 2): # Red
        curr_position.beacons_found[2] = 1
        marker.color.r = 255
        marker.color.g = 0
        marker.color.b = 0       
    elif(int(msgSplit[1]) == 3): # Yellow
        curr_position.beacons_found[3] = 1
        marker.color.r = 255
        marker.color.g = 255
        marker.color.b = 0       
    else:
        marker.color.r = 255
        marker.color.g = 255
        marker.color.b = 255
    print("Xold:", curr_position.px)
    print("Yold:", curr_position.py)
    print("Xnew:", x)
    print("Ynew:", y)
    pub.publish(marker)
    print(marker)
    #cmd_pub.publish("start")
    return

if __name__ == '__main__':
    #beacon_found = 0
    curr_position.beacons_found = [0, 0, 0, 0]
    position_list = []
    rospy.init_node('beacon_location', anonymous=True)
    rospy.Subscriber('/robot_pose', Pose, save_robot_pose)
    rospy.Subscriber('/depth_reading', String, callback)
    #rospy.Subscriber('/beacon',BLAH, callback2)
    pub = rospy.Publisher('/comp3431/beacons', Marker, queue_size=10) 
    cmd_pub = rospy.Publisher('/cmd', String, queue_size=10)
    rospy.spin()
