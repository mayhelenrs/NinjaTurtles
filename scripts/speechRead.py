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
from actionlib_msgs.msg import GoalID

global stopMessage
global goHome
global wallFollow
global isWallFollow

def callback(msg):
    global stopMessage
    global goHome
    global wallFollow
    global isWallFollow
    message = msg.data
    (tempStop, tempGoHome, tempWallFollow) = message.split(":")
    #print(tempStop, tempGoHome, tempWallFollow)
    if int(tempStop) != int(stopMessage):
        # Stop wall follow
        isWallFollow = False
        cmd_pub.publish("stop")
        cancel_nav_pub.publish(GoalID())
        tempTwist = Twist()
        tempTwist.linear.x = 0
        tempTwist.linear.y = 0
        tempTwist.linear.z = 0
        tempTwist.angular.x = 0
        tempTwist.angular.y = 0
        tempTwist.angular.z = 0
        cmd_vel_pub.publish(tempTwist)
        print("Sending stop movement",tempStop, stopMessage)
    elif (int(tempGoHome) != int(goHome)):
        # Go home
        beacon_pub.publish("1")
        print("Sending navigate home", goHome, tempGoHome)
    elif (int(tempWallFollow) != int(wallFollow)) and (isWallFollow == False):
        # Start wall follow
        isWallFollow = True
        print("lSending run wall follow", tempWallFollow, wallFollow)
        cmd_pub.publish("start")
    wallFollow = tempWallFollow
    goHome = tempGoHome
    stopMessage = tempStop

if __name__ == '__main__':
    global stopMessage
    global goHome
    global wallFollow
    global isWallFollow
    stopMessage = 0
    goHome = 0
    wallFollow = 0
    isWallFollow = False
    print("Speech command options:")
    print("\"Run wall follow\"")
    print("\"Navigate home\"")
    print("\"Stop movement\"")
    rospy.init_node('speech_read', anonymous=True)
    cmd_pub = rospy.Publisher('/cmd', String, queue_size=10)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    beacon_pub = rospy.Publisher('/beacons_found', String, queue_size=10)
    cancel_nav_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
    rospy.Subscriber('/speech', String, callback)
    rospy.spin()
