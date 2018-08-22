#!/usr/bin/env python
import datetime
import sys
from nav_msgs.msg import Odometry, Path

if not (len(sys.argv) > 1 and sys.argv[1] is "debug" ):
        import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped, Pose
from cv_bridge import CvBridge, CvBridgeError


def callback(msg):
    global xCur
    global yCur
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose = msg
    #if (xCur != pose.pose.position.x and yCur != pose.pose.position.y and yOre != pose.pose.orientation.x):
    pose.header.seq = path.header.seq + 1
    path.header.frame_id = "map"
    path.header.stamp = rospy.Time.now()
    path.header.stamp = path.header.stamp
    path.poses.append(pose)

    pub.publish(path)

    #xCur = pose.pose.position.x
    #yCur = pose.pose.position.y
    #xOre = pose.pose.orientation.x
    print("Start")
    print(path)

if __name__ == '__main__':
    beacon_found = 0
    rospy.init_node('path_publish_node', anonymous=True)
    pub = rospy.Publisher('/comp3431/path', Path, queue_size=10)
    path = Path()
    rospy.Subscriber('/robot_pose', Pose, callback)
    rospy.spin()
