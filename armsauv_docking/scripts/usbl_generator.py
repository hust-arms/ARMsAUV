#!/usr/bin/env python

import rospy
from armsauv_docking.msg import Usbl

class USBLMsgGenerator:
    def __init__(self):
        ''' Publish USBL message '''
        rospy.init_node("usbl_msg_generator")

        self.usbl_msg = Usbl()
        self.usbl_msg.header.stamp = rospy.Time.now()
        self.usbl_msg.phi = 1.658062789 # -95 degree
        self.usbl_msg.psi = -0.1001674 # -10 degree
        self.usbl_msg.s_r = 200.0
        self.usbl_msg.h_r = 198.9974874 # 
        
        # Create publisher
        self.usbl_pub = rospy.Publisher("/usbl_output", Usbl, queue_size=1)

    def msgPubThread(self):
        ''' Publish USBL message '''
        self.usbl_pub.publish(self.usbl_msg)
        rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        usbl_pub = USBLMsgGenerator()
        while not rospy.is_shutdown():
            usbl_pub.msgPubThread()
    except rospy.ROSInterruptException: pass
