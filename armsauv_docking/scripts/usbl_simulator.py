#!/usr/bin/env python

import math
import rospy
import sys
import tf
from tf import TransformListener
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from armsauv_msgs.msg import Usbl

class USBLSimulator:
    def __init__(self):
        ''' Simulate USBL message according to global position '''
        # get parameters
        self.getConfig()

        # components initialization
        rospy.init_node("usbl_simulator")

        self.tf_tree_ = TransformListener()

        rospy.Subscriber(self.input_topic_, Odometry, self.getInput)
        self.input_ = Odometry()

        self.output_pub_ = rospy.Publisher(self.output_topic_, Usbl, queue_size=1)
        self.output_ = Usbl()

    def getConfig(self):
        # get frame
        self.transc_frame_ = rospy.get_param('transceiver frame', '/armsauv/base_link')

        # get period
        self.period_ = rospy.get_param('period', 0.1)

        # get input topic
        self.input_topic_ = rospy.get_param('input topic', '/qt4/pose_gt')

        # get output topic
        self.output_topic_ = rospy.get_param('output topic', '/usbl/output')

    def getInput(self, input_pose):
        # Update input
        self.input_ = input_pose

    def getParams(self):
        # Transform pose of transponder from global frame to frame of transceiver
        input_to_transc = PoseStamped()

        input_to_global = PoseStamped()
        input_to_global.header.frame_id = self.input_.header.frame_id
        # print(self.input_.header.frame_id)
        input_to_global.header.stamp = self.input_.header.stamp
        input_to_global.pose.position = self.input_.pose.pose.position
        input_to_global.pose.orientation= self.input_.pose.pose.orientation

        self.tf_tree_.waitForTransform(self.transc_frame_, input_to_global.header.frame_id, rospy.Time.now(), rospy.Duration(0.5))
        while not rospy.is_shutdown():
            try:
                self.tf_tree_.waitForTransform(self.transc_frame_, input_to_global.header.frame_id, rospy.Time.now(), rospy.Duration(0.5))
                input_to_transc = self.tf_tree_.transformPose(self.transc_frame_, input_to_global)
            except rospy.ROSInterruptException: pass
        
        '''
        try:
            input_to_transc = self.tf_tree_.transformPose(self.transc_frame_, input_to_global)
        except rospy.ROSInterruptException: 
            pass
        '''
        input_to_transc_o = input_to_transc.pose.orientation
        input_to_transc_l = [input_to_transc_o.x, input_to_transc_o.y, input_to_transc_o.z, input_to_transc_o.w]
        euler_angle = tf.transformations.euler_from_quaternion(input_to_transc_l)
        
        x = input_to_transc.pose.position.x
        y = input_to_transc.pose.position.y
        z = input_to_transc.pose.position.z

        # get yaw to transceiver frame
        phi = euler_angle[2]

        # get euclidean distance
        dist = sqrt(x*x + y*y + z*z)

        # get horizontal plane project distance
        h_dist = sqrt(x*x + y*y)

        # get lateral angle difference
        psi = atan(z / h_dist)

        # print
        print("USBL: ", phi, psi, h_dist, dist)

        return [phi, psi, h_dist, dist]

    def outputPublish(self):
        # publish usbl messages
        output = self.getParams()
        self.output_.phi = output[0]
        self.output_.psi = output[1]
        self.output_.h_r = output[2]
        self.output_.s_r = output[3]

        self.output_pub_.publish(self.output)

        rospy.sleep(self.period_)
            
if __name__ == '__main__':
    try:
        usbl_simulator = USBLSimulator()
        rospy.sleep(3.0)

        while not rospy.is_shutdown():
            usbl_simulator.outputPublish()
    except rospy.ROSInterruptException: pass
