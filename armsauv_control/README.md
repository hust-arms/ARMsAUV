

``` bash
##armsauv_control

#rpm
rostopic pub --once /armsauv/thrusters/0/input uuv_gazebo_ros_plugins_msgs/FloatStamped '{header: auto, data: 200}'

#depth pitch yaw input
rostopic pub --once /armsauv/control_input/depth std_msgs/Float64 10
rostopic pub --once /armsauv/control_input/pitch std_msgs/Float64 0.3
rostopic pub --once /armsauv/control_input/yaw std_msgs/Float64 0.3

#depth & lateral plane control
rosrun armsauv_control armsauv_pidctrl_node

```
 
