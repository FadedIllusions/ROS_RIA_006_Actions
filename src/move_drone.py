#!/usr/bin/env python

# Import Needed Packages
import rospy
import time
import actionlib
from ardrone_as.msg import ArdroneAction, ArdroneGoal, ArdroneResult, ArdroneFeedback
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty


# Constants Corresponding To SimpleGoalState Class
PENDING = 0
ACTIVE = 1
DONE = 2
WARN = 3
ERROR = 4

# Define Number Of Images
nImage = 1


# FeedBack Callback. Called When Feedback Received From Action Server.
def feedback_callback(feedback):
    global nImage
    print('[Feedback] image n.%d received'%nImage)
    nImage += 1


# Init Action Client Node
rospy.init_node('ex_action_client_node')

# Define Server Name
action_server_name = '/ardrone_action_server'

# Instatiate Client -- Server Name, Action
client = actionlib.SimpleActionClient(action_server_name, ArdroneAction)

# Create Publisher To Move Drone
move = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
move_msg = Twist()

# Create Publisher To Takeoff Drone
takeoff = rospy.Publisher('/drone/takeoff', Empty, queue_size=1)
takeoff_msg = Empty()

# Create Publisher To Land Drone
land = rospy.Publisher('/drone/land', Empty, queue_size=1)
land_msg = Empty()

# Wait For Server
rospy.loginfo("Waiting For Action Server" +action_server_name)
client.wait_for_server()
rospy.loginfo("Action Server Found")

# Instantiate Goal
goal = ArdroneGoal()
goal.nseconds = 10

# Send Goal To Action Server, Specify Feedback Callback
client.send_goal(goal, feedback_cb=feedback_callback)

# Get State Of Action
state_result = client.get_state()
rate = rospy.Rate(1)
rospy.loginfo("state_result: " +str(state_result))

# Drone Takeoff -- First 3 Seconds
i = 0
while not i==3:
    takeoff.publish(takeoff_msg)
    rospy.loginfo("Taking Off...")
    time.sleep(1)
    i += 1

    # Fly Drone In Circle
while state_result < DONE:
    move_msg.linear.x = 1
    move_msg.angular.z = 1
    move.publish(move_msg)

    rate.sleep()

    # Update State Result
    state_result = client.get_state()

    rospy.loginfo("Moving Around...")
    rospy.loginfo("state_result: " +str(state_result))

rospy.loginfo("[Result] State: " +str(state_result))

if state_result == ERROR:
    rospy.logerr("Error On Server Side")
if state_result == WARN:
    rospy.logwarn("Warning On Server Side")

# Land Drone
i=0
while not i==3:
    move_msg.linear.x = 0
    move_msg.angular.z = 0
    move.publish(move_msg)
    land.publish(land_msg)

    rospy.loginfo("Landing...")
    time.sleep(1)

    i += 1