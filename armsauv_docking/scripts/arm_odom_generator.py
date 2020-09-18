#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

class ArmOdomGenerator:
    def __init__(self):
        ''' Simulate odometry message generator for manipulator '''
        rospy.init_node("arm_odom_generator")
 
        self.arm_pose = Odometry()
        self.topic = "/arm/pose_gt"
        self.odom_pub = rospy.Publisher(self.topic, Odometry, queue_size=1)
        
        # Set value
        self.arm_pose.pose.pose.position.x = 5.0
        self.arm_pose.pose.pose.position.y = 5.0
        self.arm_pose.pose.pose.position.z = 0.0
        self.arm_pose.pose.pose.orientation.x = 0.0
        self.arm_pose.pose.pose.orientation.y = 0.0
        self.arm_pose.pose.pose.orientation.z = 0.0
        self.arm_pose.pose.pose.orientation.w = 1.0

    def pubOdomThread(self):
        ''' Publish odometry '''
        self.odom_pub.publish(self.arm_pose)
        rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        arm_odom_pub = ArmOdomGenerator()
        while not rospy.is_shutdown():
            arm_odom_pub.pubOdomThread()
    except rospy.ROSInterruptException: pass
