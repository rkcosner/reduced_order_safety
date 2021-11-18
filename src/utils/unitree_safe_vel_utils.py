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

# Store Parameters in Dict
par = {
    'xgoal' : xgoal, 
    'xO'    : xO, 
    'DO'    : DO, 
    'Kp'    : Kp, 
    'alpha' : alpha, 
    'Kv'    : Kv, 
    'dim'   : dim,
    'delta' : delta,
    'Kom'   : Kom,
    'R'     : R
}

def setupSOCP(barrier_bits, u_des):
    G = [[-1/np.sqrt(2), 0, 0], 
    [-1/np.sqrt(2), 0, 0 ], 
    [0, -1, 0], 
    [0, 0, -R]]
        
    b = [1/np.sqrt(2), 
        -1/np.sqrt(2), 
        0, 
        0]
    cones = [4] 

    for bit in barrier_bits: 
        cones.append(3)
        h = bit[0]
        Lfh = bit[1]
        Lgh = bit[2]
        LghLgh = bit[3]
        G.append([0, -Lgh[0,0],  -Lgh[0,1]])
        G.append([0, -self.gamma*self.L_lgh, 0])
        G.append([0, 0, -self.gamma*self.L_lgh])
        b.append( (Lfh + par['alpha']*h - (self.L_lfh + self.sigma*self.L_lgh + self.L_ah)*self.gamma - self.sigma*LghLgh).item() )
        b.append(0)
        b.append(0)
    G = sp.sparse.csc_matrix(G)
    b = np.array(b)

    SOCP_dims = {
        'l':0,      # linear cone size
        'q':cones,  # second order cone size
        'e':0       # exponential cone sizes
    }

    cost = np.array([1.0, -u_des[0].item(),  R**2*-u_des[1].item()])

    return G, b, cones, SOCP_dims, cost