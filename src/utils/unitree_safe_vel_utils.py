#!/usr/bin/env python
import rospy
import rospkg
import sys 
rospack = rospkg.RosPack()
sys.path.append(rospack.get_path('red_ord_unitree')+'/src/utils/')
from general import *


# Standard Imports
from datetime import datetime
import os 
import numpy as np 
import scipy as sp 

# Messages 
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
import std_msgs.msg

# SOCP Solver
import ecos 


###########################################################
# Problem Params
xgoal = np.array([[4,-1]]).T
dim = len(xgoal)
xO = np.empty((0,0)) #np.array([[1.5,0],[3, -1.5]]).T
DO = np.empty((0))

# Controller Params
scale = 0.5
Kp = 0.2*scale
Kv = 0.08*scale
delta = 0.1
Kom = 0.4*scale
R = 0.25

alpha = 10

visual_offset = 0.26 #m 
