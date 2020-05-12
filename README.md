# ROS_RIA_006_Actions
In this example, we are using a simulated AR Parrot Drone to explain and implement ROS actions. Actions are quite 
similar to services wherein, when you call an action, you are calling a functionality provided by another node. 
However,unlike with services, actions needn't wait until the service has completed.

Our code is going to command the Parrot drone to fly in a circle whilst taking pictures every second.

Like services, an action must have both an action server and an action client.

To execute the current repo:
```roslaunch ardrone_as action_server.launch```
```roslaunch action_ex_pkg move_drone.launch```

(See [NOTES.md](https://github.com/FadedIllusions/ROS_RIA_006_Actions/edit/master/NOTES.md) for section notes.)
