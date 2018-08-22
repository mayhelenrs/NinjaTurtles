#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from actionlib_msgs.msg import GoalID
import threading
from pprint import pprint

#this lets us send messages onn the /cmd topic to 
#start and stop wall following
cmd_pub = rospy.Publisher('/cmd', String, queue_size=10)

#this lets us publish a goal destination
#to send the robot back home
nav_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
cancel_nav_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)

starting_pose = None

found_beacons = False

positions_in_time = [None]*10




def get_current_pose():
    pose = rospy.wait_for_message('robot_pose', Pose)
    return pose


def commence_navigation():
    global starting_pose
    starting_pose = get_current_pose()
    cmd_pub.publish("start")

def go_home():
    #stop wallfollowing
    rospy.logwarn("Going home")
    cmd_pub.publish("stop")
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "map"
    pose.pose = starting_pose
    nav_pub.publish(pose)

    log_locations()


def command_callback(data):
    command = data.data
    if command == "go":
        print "starting"
        commence_navigation()
    elif command == "go_home":
        print "done"
        global found_beacons
        found_beacons = True
        go_home()
    else:
        print(command)



def perform_emergency_wallfollow():
    #we are stuck
    #we will wallfollow for 5 seconds
    #start wallfollowing:
    cancel_nav_pub.publish(GoalID())
    cmd_pub.publish("start")

    #5 seconds later, go home
    threading.Timer(5.0, go_home).start()

def log_locations():
    position = rospy.wait_for_message('robot_pose', Pose).position
    global positions_in_time
    positions_in_time = positions_in_time[-1:] + positions_in_time[:-1]
    positions_in_time[0] = position

    ten_seconds_ago = positions_in_time[9]
    current_position = positions_in_time[0]

    if ten_seconds_ago is not  None and (round(current_position.x, 2) == round(ten_seconds_ago.x, 2)) and (round(current_position.y, 2) == round(ten_seconds_ago.y, 2)):
        #then we havent move more than a cm in 10 seconds
        #we should give up and wallfollow
        
        #we dont want to perform emergency manouvres if we're at the goal
        if (abs(current_position.x - starting_pose.position.x) < 20) and (abs(current_position.y - starting_pose.position.y) < 20):
            return
        
        
        rospy.logwarn("we need to take emergency action!")
        perform_emergency_wallfollow()
        return
    else:
        rospy.logwarn("In my opinion, we have moved in the last 10 seconds")
        threading.Timer(1.0, log_locations).start()
    



def pose_callback(pose):
    position = pose.position

    rospy.logwarn(pose)

def beacons_have_been_found(message):
    global found_beacons
    if found_beacons:
        return
    
    message = message.data

    if message == "1":
        global found_beacons
        found_beacons = True
        rospy.logerr("Found all eacons! time for home!")
        go_home()

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)


    rospy.Subscriber('cmd', String, command_callback)

    rospy.Subscriber('beacons_found', String, beacons_have_been_found)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
