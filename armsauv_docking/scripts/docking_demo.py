#!/usr/bin/env python

import math
import sys
from nav_msgs.msg import odometry
from std_msgs.msg import float64
from tf.transformations import euler_from_quaternion

from numpy import*

class AUVObserver:
    def __init__(self):
        ''' Effect behavior of auv '''

    def getConfig(self):
        ''' Load parameters from rosparam server '''
        self.

