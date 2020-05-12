AR Drone Takeoff
----------------
```rostopic pub /drone/takeoff std_msgs/Empty "{}" ```
(CTRL+C to stop drone and be able to further issue commands.)


Teleop Twist Keyboard
---------------------
```rosrun teleop_twist_keyboard teleop_twist_keyboard.py```


AR Drone Land
-------------
```rostopic pub /drone/land std_msgs/Empty "{}" ```


List ROS Topics
---------------
```rostopic list```


List ROS Services
-----------------
```rosservice list```


List ROS Actions
----------------
```rostopic list```



As seen in the example provided, the following topics were listed (Every 
action server will create these 5 topics.):

```
/ardrone_action_server/cancel
/ardrone_action_server/feedback
/ardrone_action_server/goal
/ardrone_action_server/result
/ardrone_action_server/status
```

Wherein 'ardrone_action_server' is the name of the action server and 
'cancel', 'feedback', 'goal', 'result', and 'status' are messages used to 
communicate with the action server.



As with services, calling an action means that you are sending a message to
that action.
  * The message of a topic is composed of a single part: the information 
    the topic provides
  * The message of a service is composed of two parts: the request and the
    response
  * The message of an action server is divided into three parts: the goal,
    the result, and the feedback.

Let's look at an example:
```roscd ardrone_as/action; cat Ardrone.action```

```
  # Goal For Drone
  int32 nseconds   # Seconds Drone Will Take Pictures
  ---
  sensor_msgs/CompressedImage[] allPictures   # Array Of All Pics Taken
                                              # In nseconds
  ---
  sensor_msgs/CompressedImage lastImage   # Last Image Taken
```



Like with a service, you call an action server by implementing an action
client. Let's look at an example:

```
#! /usr/bin/env python

# Import Needed Packages
import rospy
import time
import actionlib
from ardrone_as.msg import ArdroneAction, ArdroneGoal, ArdroneResult, ArdroneFeedback


# Define Number Of Images
nImage = 1


# FeedBack Callback. Called When Feedback Received From Action Server.
# Prints Message Indicating New Message Received.
def feedback_callback(feedback):
    global nImage
    print('[Feedback] image n.%d received'%nImage)
    nImage += 1


# Init Action Client Node
rospy.init_node('drone_action_client')

# Instantiate Client -- Initiate Connection
# Wait For Server
# First Parameter: Action Server To Connect To
# Second Parameter: Type Of Action Message
# Note Naming Convention -- File: Ardrone.action, Parameter: ArdroneAction
client = actionlib.SimpleActionClient('/ardrone_action_server', ArdroneAction)
client.wait_for_server()

# Instantiate Goal
# Again, Note Naming Convention
goal = ArdroneGoal()
goal.nseconds = 10   # Take Pics 10 Seconds

# Send Goal To Action Server, Specify Feedback Callback
client.send_goal(goal, feedback_cb=feedback_callback)

# Uncomment To Test Goal Preemption:
#time.sleep(3.0)
#client.cancel_goal()  # Cancel Goal After 3 Seconds

# Await Result
# status = client.get_state()
client.wait_for_result()

print('[Result] State: %d'%(client.get_state()))
```


Now, we know how to call an action and await a result, as we would with a service; however, let's use an action more for what it's intended -- Not having to wait for it to complete.

Notice above, the ```client.wait_for_result()```. This simple function waits until the action has completed and returns a boolean value. However, with such a function, we're locked into having to wait for the action to complete, much as we would have to when calling a service.

The SimpleActionClient has another method, ```get_state()```, that returns an int value denoting which state the SimpleActionClient object is in:

  0 ==> PENDING
  1 ==> ACTIVE
  2 ==> DONE
  3 ==> WARN
  4 ==> ERROR

This would allow us to create a while loop to check the returned value of the get_state method and, so long as the return isn't an int value of 2 or higher, the action is still in progress and we can continue doing other things.



Let's see how we would continue doing things whilst awaiting the action to complete. Like with a service server, to launch an action server, we'd command similar to ```roslaunch ardrone_as action_server.launch```.

```
#!/usr/bin/env python

# Import Needed Packages
import rospy
import time
import actionlib
from ardrone_as.msg import ArdroneAction, ArdroneGoal, ArdroneResult, ArdroneFeedback



"""
class SimpleGoalState:
    PENDING = 0
    ACTIVE = 1
    DONE = 2
    WARN = 3
    ERROR = 4

"""

# Create Constants With Values Corresponding To SimpleGoalState
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
rospy.init_node('example_no_waitforresult_action_client_node')

# Define Server Name
action_server_name = '/ardrone_action_server'

# Instatiate Client -- Server Name, Action
# Wait For Server
client = actionlib.SimpleActionClient(action_server_name, ArdroneAction)
rospy.loginfo('Waiting For Action Server '+action_server_name)
client.wait_for_server()
rospy.loginfo('Action Server Found...'+action_server_name)

# Instantiate Goal
goal = ArdroneGoal()
goal.nseconds = 10

# Send Goal To Action Server, Specify Feedback Callback
client.send_goal(goal, feedback_cb=feedback_callback)


# Get State Of Action
state_result = client.get_state()
rate = rospy.Rate(1)

# Print State Result
rospy.loginfo("state_result: "+str(state_result))

# Until State Done
while state_result < DONE:
    rospy.loginfo("Doing Stuff Whilst Awaiting Server Result...")
    rate.sleep()
    state_result = client.get_state()
    rospy.loginfo("state_result: "+str(state_result))
    
rospy.loginfo("[Result] State: "+str(state_result))

if state_result == ERROR:
    rospy.logerr("Error On Server Side...")
if state_result == WARN:
    rospy.logwarn("Warning On Server Side...")
```

