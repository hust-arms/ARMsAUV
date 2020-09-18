#!/usr/bin/env python

import math
import rospy
import sys
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from armsauv_docking.msg import Usbl
from tf.transformations import euler_from_quaternion

from numpy import*

class Observer:
    def __init__(self):
        ''' simulate USBL behavior '''
        self.auv_ns = sys.argv[1]
        self.arm_ns = sys.argv[2]

        # get configuration
        self.getConfig()

        # Create publisher
        self.usbl_pub = rospy.Publisher(self.usbl_output_topic, Usbl, queue_size=1)
        
        rospy.init_node("docking_observer")

        # Create subscriber
        rospy.Subscriber(self.auv_pose_topic, Odometry, self.getAUVPose)
        rospy.Subscriber(self.arm_pose_topic, Odometry, self.getARMPose)

        # Create message
        self.auvPose = Odometry()
        self.armPose = Odometry()

    def getConfig(self):
        ''' load paramters from the rosparam server '''
        self.auv_pose_topic = "/" + self.auv_ns + "/pose_gt"
        self.arm_pose_topic = "/" + self.arm_ns + "/pose_gt"
        self.usbl_output_topic = "/usbl_output"
        self.period = 0.1
        # self.auv_pose_topic = rospy.get_param(self.auv_ns + "/pose_gt")
        # self.arm_pose_topic = rospy.get_param(self.arm_ns + "/pose_gt")
        # self.usbl_output_topic = rospy.get_param("/usbl_output") 
        # self.period = rospy.get_param(0.1) # period: s

    def getAUVPose(self, pose):
        self.auvPose = pose

    def getARMPose(self, pose):
        self.armPose = pose

    def getUSBLMsg(self):
        x_auv = self.auvPose.pose.pose.position.x
        y_auv = self.auvPose.pose.pose.position.y
        z_auv = self.auvPose.pose.pose.position.z
        x_arm = self.armPose.pose.pose.position.x
        y_arm = self.armPose.pose.pose.position.y
        z_arm = self.armPose.pose.pose.position.z

        ''' compute distance between auv and arm '''
        delta_x = x_auv - x_arm
        delta_y = y_auv - y_arm
        delta_z = z_auv - z_arm
        dist = sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z)

        ''' compute lateral angle differential '''
        lat_ang = math.asin((z_auv - z_arm) / dist)
        
        ''' compute horizontal angle differential '''
        hor_ang = math.atan2(y_auv, x_auv) - math.atan2(y_arm, x_arm) 

        ''' compute project distance '''
        project_d = dist * cos(lat_ang)

        return [hor_ang, lat_ang, project_d, dist]
    
    def usblPubThread(self):
        usbl_msg = Usbl()

        usbl_l = self.getUSBLMsg()
        
        usbl_msg.phi = usbl_l[0]
        usbl_msg.psi = usbl_l[1]
        usbl_msg.h_r = usbl_l[2]
        usbl_msg.s_r = usbl_l[3]

        self.usbl_pub.publish(usbl_msg)

        rospy.sleep(self.period)


if __name__ == '__main__':
    try:
        docking_observer = Observer()
        rospy.sleep(5.0)
        while not rospy.is_shutdown():
            docking_observer.usblPubThread()
    except rospy.ROSInterruptException: pass

