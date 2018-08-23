#!/usr/bin/env python
import time
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
    print(curr_position.beacons_found[0],curr_position.beacons_found[1],curr_position.beacons_found[2],curr_position.beacons_found[3],curr_position.beacons_found[4])
    if ((curr_position.beacons_found[0]+curr_position.beacons_found[1]+curr_position.beacons_found[2]+curr_position.beacons_found[3]) >= 4):
        beacons_found.publish("1")
        #curr_position.beacons_found[0] = 0
        #curr_position.beacons_found[1] = 0
        #curr_position.beacons_found[2] = 0
        #curr_position.beacons_found[3] = 0
        #curr_position.beacons_found[4] = 0
        #print(curr_position.beacons_found) 
        print("Done")
    else:
        beacons_found.publish("0")
    curr_position.timestamp = time.time()
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
    msgSplit = msg.data.split()

    min_time = None
    bRobot_position = curr_position
    for i in position_list:
        temp = float(msgSplit[2]) - i.timestamp
        if (min_time is None) or (abs(temp) < min_time):
            min_time = abs(temp)
            bRobot_position = i
            temp_time = i.timestamp

    quaternion = (bRobot_position.ox, bRobot_position.oy, bRobot_position.oz, bRobot_position.ow)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    print("time",bRobot_position.timestamp,":",temp_time)
    #print("roll", euler[0])
    #print("pitch", euler[1])
    #print("yaw", euler[2])

    print(msgSplit)
    depth = (float(msgSplit[0])/1000) - 0.15
    offset = 0.001
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
        if (curr_position.beacons_found[0] != 1):
            curr_position.beacons_found[0] = 1
            marker.color.r = 0
            marker.color.g = 255
            marker.color.b = 0
            pub.publish(marker)
    elif(int(msgSplit[1]) == 1): # Blue
        if (curr_position.beacons_found[1] != 1):
            curr_position.beacons_found[1] = 1
            marker.color.r = 0
            marker.color.g = 0
            marker.color.b = 255
            pub.publish(marker)
    elif(int(msgSplit[1]) == 2): # Red
        if (curr_position.beacons_found[2] != 1):
            curr_position.beacons_found[2] = 1
            marker.color.r = 255
            marker.color.g = 0
            marker.color.b = 0
            pub.publish(marker)
    elif(int(msgSplit[1]) == 3): # Yellow
        if (curr_position.beacons_found[3] != 1):
            curr_position.beacons_found[3] = 1
            marker.color.r = 255
            marker.color.g = 255 
            marker.color.b = 0
            pub.publish(marker)
    else:
        if (curr_position.beacons_found[4] != 1):
            curr_position.beacons_found[4] = 1
            marker.color.r = 255
            marker.color.g = 255
            marker.color.b = 255
            pub.publish(marker)
    return

if __name__ == '__main__':
    #beacon_found = 0
    curr_position.beacons_found = [0, 0, 0, 0, 0]
    #curr_position.beacons_found = [1, 1, 1, 1, 1]
    print(curr_position.beacons_found)
    time.sleep(3)
    position_list = []
    rospy.init_node('beacon_location', anonymous=True)
    rospy.Subscriber('/robot_pose', Pose, save_robot_pose)
    rospy.Subscriber('/depth_reading', String, callback)
    beacons_found = rospy.Publisher('/beacons_found',String, queue_size=10)
    pub = rospy.Publisher('/comp3431/beacons', Marker, queue_size=10) 
    rospy.spin()
