#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

class AuvOdomGenerator:
    def __init__(self):
        ''' Simulate odometry message generator for auv'''
        rospy.init_node("auv_odom_generator")
 
        self.auv_pose = Odometry()
        self.topic = "/armsauv/pose_gt"
        self.odom_pub = rospy.Publisher(self.topic, Odometry, queue_size=1)
        
        # Set value
        self.auv_pose.pose.pose.position.x = 5.0
        self.auv_pose.pose.pose.position.y = 8.66
        self.auv_pose.pose.pose.position.z = -5.0
        self.auv_pose.pose.pose.orientation.x = 0.0
        self.auv_pose.pose.pose.orientation.y = 0.0
        self.auv_pose.pose.pose.orientation.z = 0.0
        self.auv_pose.pose.pose.orientation.w = 1.0

    def pubOdomThread(self):
        ''' Publish odometry '''
        self.odom_pub.publish(self.auv_pose)
        rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        auv_odom_pub = AuvOdomGenerator()
        while not rospy.is_shutdown():
            auv_odom_pub.pubOdomThread()
    except rospy.ROSInterruptException: pass
