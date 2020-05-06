
``` bash
rostopic pub --once /armsauv/fins/3/input uuv_gazebo_ros_plugins_msgs/FloatStamped '{header: auto, data: 0.5}'
rostopic pub --once /armsauv/thrusters/0/input uuv_gazebo_ros_plugins_msgs/FloatStamped '{header: auto, data: 200}'
```
