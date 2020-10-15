#!/usr/bin/env python

import math
import sys
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion

from numpy import*

class DockingObserver:
    def __init__(self):
        ''' Effect behavior of auv '''
        self.getConfig()

        rospy.init_node("docking_demo")

        rospy.Subscriber(self.auv_odom_t_, Odometry, self.auvOdomCb)

        rospy.Subscriber(self.arm_odom_t_, Odometry, self.armOdomCb)

        self.is_auv_odom_ = False
        self.is_arm_odom_ = False

        self.arm_mission_pub_ = rospy.Publisher(self.arm_mission_t_, String, queue_size=1)

        self.auv_mission_pub_ = rospy.Publisher(self.auv_mission_t_, String, queue_size=1)

        self.auv_odom_ = Odometry()

        self.arm_odom_ = Odometry()

    def getConfig(self):
        ''' Load parameters from rosparam server '''
        self.auv_odom_t_ = rospy.get_param("auv_odom_topic", "/armsauv/base_link")
        self.arm_odom_t_ = rospy.get_param("arm_odom_topic", "/uvsm719/base_link")

        self.arm_mission_t_ = rospy.get_param("arm_mission_topic", "/arm_mission")
        self.auv_mission_t_ = rospy.get_param("auv_mission_topic", "/auv_mission")
        self.period_ = rospy.get_param("period", 0.1)

        self.auv_threshold_ = rospy.get_param("auv_threshold", 15.0)

        self.arm_threshold1_ = rospy.get_param("arm_threshold1", 30.0)

        self.arm_threshold2_ = rospy.get_param("arm_threshold2", 5.0)

        self.arm_threshold3_ = rospy.get_param("arm_threshold3", 0.5)

        self.auv_mission_ = rospy.get_param("auv_mission", "LowVelocity")

        self.arm_mission1_ = rospy.get_param("arm_mission1", "StrenchOut")

        self.arm_mission2_ = rospy.get_param("arm_mission2", "MoveToAUV")
        self.arm_mission3_ = rospy.get_param("arm_mission3", "GraspAUV")


    def auvOdomCb(self, odom):
        ''' AUV odometry Callback function '''
        self.is_auv_odom_ = True
        self.auv_odom_ = odom 

    def armOdomCb(self, odom):
        ''' Arm odometry Callback function '''
        self.is_arm_odom_ = True
        self.arm_odom_ = odom

    def getDistance(self):
        ''' Compute euclidean distance between AUV and Arm '''
        delta_x = auv_odom_.pose.pose.position.x - arm_odom_.pose.pose.position.x
        delta_y = auv_odom_.pose.pose.position.y - arm_odom_.pose.pose.position.y
        delta_z = auv_odom_.pose.pose.position.z - arm_odom_.pose.pose.position.z
        return sqrt(delta_x*delta_x + delta_y*delta_y + delta_z*delta_z)

    def observeDocking(self):
        ''' Publish mission '''
        try:
            if self.is_auv_odom_ and self.is_arm_odom_:
                dist = self.getDistance()
                if dist <= auv_threshold_:
                    auv_mission_pub_.publish(self.auv_mission_)
                    print("[docking_demo]:auv should execute mission")
                if dist <= arm_threshold1_ and dist > arm_threshold2_:
                    arm_mission_pub_.publish(self.arm_mission1_)
                    print("[docking_demo]:arm should execute mission1")
                if dist <= arm_threshold2_ and dist > arm_threshold3_:
                    arm_mission_pub_.publish(self.arm_mission2_)
                    print("[docking_demo]:arm should execute mission2")
                if dist <= arm_threshold3_:
                    arm_mission_pub_.publish(self.arm_mission3_)
                    print("[docking_demo]:arm should execute mission3")
            else print("[docking_demo]:Waiting for odometry messages")

        except rospy.ROSInterruptException: pass


if __name__ == '__main__':
    try:
        docking_observer = DockingObserver()
        
        while not rospy.is_shutdown():
            docking_observer.observeDocking()
            rospy.sleep(self.period_)

    except rospy.ROSInterruptException: pass




