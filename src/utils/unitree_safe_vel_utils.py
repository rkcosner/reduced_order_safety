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
xO = np.array([[1.5,0],[3, -1.5]]).T# np.empty((0,0)) #
DO = 0.5#np.empty((0))

# Controller Params
scale = 0.5
Kp = 0.2*scale
Kv = 0.08*scale
delta = 0.25
Kom = 0.4*scale
R = 0.25


visual_offset = 0.26 #m 

# Store Parameters in Dict
par = {
    'xgoal' : xgoal, 
    'xO'    : xO, 
    'DO'    : DO, 
    'Kp'    : Kp, 
    'Kv'    : Kv, 
    'dim'   : dim,
    'delta' : delta,
    'Kom'   : Kom,
    'R'     : R
}

def K_CBF_SOCP(barrier_bits, u_des, L_ah, L_lfh, L_lgh, L_lgh2, sigma, gamma, alpha):
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
        G.append([0, -gamma*L_lgh, 0])
        G.append([0, 0, -gamma*L_lgh])
        b.append( (Lfh + alpha*h - (L_lfh + sigma*L_lgh + L_ah)*gamma - sigma*LghLgh).item() )
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

    ecos_solver_output = ecos.solve(cost, G, b, SOCP_dims, verbose=False)

    if ecos_solver_output['info']['exitFlag'] ==0 or ecos_solver_output['info']['exitFlag'] ==10: 
        # ECOS Solver Successful
        return np.expand_dims(ecos_solver_output['x'][1:3],1)
    else: 
        # ECOS Solver Failed 
        rospy.logwarn('SOCP failed') # Filter falls back to zero input

def measureCBFutil(z): 
    dim = par['dim']
    xO = par['xO']
    DO = par['DO']
    delta = par['delta']

    x = z[0:dim,:]
    psi = z[dim,:][0]

    tpsi = np.array([[np.cos(psi), np.sin(psi)]]).T

    # Control Barrier Function 
    hk = np.zeros((xO.shape[1],1))
    for kob in range(xO.shape[1]): 
        xob = np.array([xO[:,kob]]).T
        robs = DO
        dobs = np.linalg.norm(x-xob)
        nobs = (x - xob)/dobs
        nobstpsi = nobs.T@tpsi
        hk[kob] = dobs - robs +delta*nobstpsi

    # Use only the closest obstacle
    if len(hk) > 0: 
        h = hk.min()
        idx = hk.argmin()
        xob = np.array([xO[:,idx]]).T
        d = np.linalg.norm(x - xob)
        r = DO
        nO = (x - xob)/d 
        nOtpsi = nO.T@tpsi #np.dot(nO.T, tpsi)
        h = d - r + delta*nOtpsi[0,0]
        return h
    else: 
        return 0.42

def K_des(z):
    xgoal = par['xgoal']
    dim = par['dim']
    Kv = par['Kv']    
    Kom = par['Kom']
    x = z[0:dim,:]
    psi = z[dim,:][0]
    udes = np.array([[Kv*np.linalg.norm(x-xgoal), 
                    -Kom*(np.sin(psi) - (xgoal[1][0]-x[1][0])/np.linalg.norm(x-xgoal) )]]).T

    return udes
        

def saturate(input): 
    max_input = [0.5, 1]
    for i in range(2):
        if np.abs(input[i,0])>max_input[i]:
            input[i,0] = max_input[i]*np.sign(input[i,0])
    return input