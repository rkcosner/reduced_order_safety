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
xgoal = np.array([[4,-1.5]]).T
dim = len(xgoal)
xO =  np.empty((0,0)) #
xOsim = np.array([[1.5,0],[3, -2]]).T#
DO = 0.5 + robot_radius#np.empty((0))

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

def K_CBF_SOCP(barrier_bits, u_des, alpha, sigma, MRCBF_add, MRCBF_mult):
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
        G.append([0, -MRCBF_mult, 0])
        G.append([0, 0, -MRCBF_mult])
        b.append( (Lfh + alpha*h - (sigma*MRCBF_mult+MRCBF_add) - sigma*LghLgh).item() )
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
        return np.array([[0,0]]).T

def measureCBFutil(z, xO): 
    dim = par['dim']
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




class safe_velocity_node(): 

    def __init__(self, experiment_type, A, B, C, D): 
        rospy.init_node('safe_velocity', anonymous=True)

        self.flag_state_received = False

        self.pub = rospy.Publisher('cmd', Twist, queue_size=1)

        if experiment_type == 0: #Sim 
            # Use default barrier positions (in unitree_safe_vel_utils) and use true sim position
            par["xO"] = xOsim
            rospy.Subscriber('unitreeOnboardStates', Twist, self.stateCallback)
        elif experiment_type == 1: # In lab  
            # Use measured barrier positions and SLAM. Also, record ground truth  
            rospy.Subscriber("/vrpn_client_node/Unitree/pose", PoseStamped, self.unitreeMocapCallback)
            rospy.Subscriber("/vrpn_client_node/Tower1/pose", PoseStamped, self.tower1Callback)
            rospy.Subscriber("/vrpn_client_node/Tower2/pose", PoseStamped, self.tower2Callback)
            rospy.Subscriber("/vrpn_client_node/Tower3/pose", PoseStamped, self.tower3Callback)
            rospy.Subscriber('barrier_IDs', MarkerArray, self.barrierIDsCallback)
            rospy.Subscriber('t265/odom/sample', Odometry, self.slamCallback) 
        elif experiment_type == 2: # Outside
            # Use measured barrier positions and SLAM.
            rospy.Subscriber('barrier_IDs', MarkerArray, self.barrierIDsCallback)
            rospy.Subscriber('t265/odom/sample', Odometry, self.slamCallback) 

        self.experiment_type = experiment_type
        

        self.state = np.array([[0.,0.,0.]]).T
        self.cmdVel = Twist()

        # record values 
        self.u_traj = []
        self.x_traj = []
        self.x_mocap_traj = []
        self.u_des_traj = []
        self.h_meas_traj = []
        self.h_true_traj = []
        self.obs_traj = [par["xO"]]
        self.tower1_mocap_pose = np.empty((0,0))
        self.tower2_mocap_pose = np.empty((0,0))        
        self.tower3_mocap_pose = np.empty((0,0))     
        if self.experiment_type == 0:
            self.tower1_mocap_pose = [xOsim[0][0],xOsim[1][0] ]
            self.tower2_mocap_pose = [xOsim[0][1],xOsim[1][1] ]
        
        self.alpha = A
        self.sigma = B
        self.MRCBF_add = C
        self.MRCBF_mult = D



    def stateCallback(self, data): 
        if self.flag_state_received == False: 
            self.flag_state_received = True

        self.state[0,0] = data.linear.x
        self.state[1,0] = data.linear.y
        self.state[2,0] = data.angular.z
        
        # data logging
        self.x_traj.append([data.linear.x, data.linear.y, data.linear.z])
        

        self.obs_traj.append(par['xO'])


    def slamCallback(self, data): 
        if self.flag_state_received == False: 
            self.flag_state_received = True
        

        q_z = data.pose.pose.orientation.z
        q_w = data.pose.pose.orientation.w
        yaw = np.arctan2(2*q_w*q_z, 1-2*q_z**2)

        self.state[0,0] = data.pose.pose.position.x - realsense_offset*np.cos(yaw) + realsense_offset
        self.state[1,0] = data.pose.pose.position.y - realsense_offset*np.sin(yaw) 
        self.state[2,0] = yaw

        self.x_traj.append([self.state[0,0], self.state[1,0], self.state[2,0]])


    def barrierIDsCallback(self, data):
        ## Receive and Update Barrier Positions
        # - reads barrier_IDs message 
        # - resets xO to the measured positions
        # - sets Dob to be the appropriate length 
        # - records obstacle locations

        print("new barriers")
        xO = []
        for marker in data.markers: 
            pose = [marker.pose.position.x, marker.pose.position.y]
            xO.append(pose)
            self.obs_traj.append(pose)
        xO = np.array(xO).T
        if len(xO) == 0: 
            xO = np.empty((0,0))
        par["xO"] = xO
        par["DO"] = (0.5+robot_radius)
        print(xO)

    def tower1Callback(self, data):
        if len(self.tower1_mocap_pose) == 0: 
            self.tower1_mocap_pose = [data.pose.position.x, data.pose.position.y]
    def tower2Callback(self, data):
        if len(self.tower2_mocap_pose) == 0: 
            self.tower2_mocap_pose = [data.pose.position.x, data.pose.position.y]
    def tower3Callback(self, data):
        if len(self.tower3_mocap_pose) == 0: 
            self.tower3_mocap_pose = [data.pose.position.x, data.pose.position.y]
    def unitreeMocapCallback(self, data): 
        q_z = data.pose.orientation.z
        q_w = data.pose.orientation.w
        yaw = np.arctan2(2*q_w*q_z, 1-2*q_z**2)
        self.x_mocap_traj.append([data.pose.position.x, data.pose.position.y, yaw])


    def pubCmd(self):
        if self.flag_state_received == False: 
            return 

        # Get Desired Input
        u_des = K_des(self.state)
        self.u_traj.append(u_des)

        # Filter through CBF SOCP
        barrier_bits = self.getBarrierBits()

        u_np = K_CBF_SOCP(barrier_bits,u_des, self.alpha, self.sigma, self.MRCBF_add, self.MRCBF_mult)

        # Publish v_cmd
        self.cmdVel.angular.z = u_np[1,0]

        if self.experiment_type == 0: 
            # Add drift experimental
            self.cmdVel.linear.x = u_np[0,0]+0.05
        else:        
            self.cmdVel.linear.x = u_np[0,0]


        u_np = saturate(u_np)

        self.pub.publish(self.cmdVel)

        # Log Data
        self.u_des_traj.append(u_np[0:2])
        self.measureCBF()

    def measureCBF(self): 
        # Measured CBF Value 
        z = self.state
        h = measureCBFutil(z, par["xO"])
        self.h_meas_traj.append(h)

        if self.experiment_type == 1 or self.experiment_type == 0 : 
            # If in lab with mocap record ground truth
            towers = [self.tower1_mocap_pose, self.tower2_mocap_pose, self.tower3_mocap_pose]
            xO_true = []
            for t in towers: 
                if len(t) > 0: 
                    xO_true.append(t)
            xO_true = np.asarray(xO_true).T
            self.h_true_traj.append(measureCBFutil(z, xO_true))

    def getBarrierBits(self):
        z = self.state
        dim = par['dim']
        xO = par['xO']
        DO = par['DO']
        delta = par['delta']

        x = z[0:dim,:]
        psi = z[dim,:][0]

        tpsi = np.array([[np.cos(psi), np.sin(psi)]]).T
        npsi = np.array([[-np.sin(psi),np.cos(psi)]]).T

        barrier_bits = []

        # Control Barrier Function 
        hk = np.zeros((xO.shape[1],1))
        for kob in range(xO.shape[1]): 
            xob = np.array([xO[:,kob]]).T
            robs = DO
            dobs = np.linalg.norm(x-xob)

            nobs = (x - xob)/dobs
            nobstpsi = nobs.T@tpsi
            hk[kob] = dobs - robs +delta*nobstpsi

            xob = np.array([xO[:,kob]]).T
            d = np.linalg.norm(x - xob)
            r = DO
            nO = (x - xob)/d 
            nOtpsi = nO.T@tpsi #np.dot(nO.T, tpsi)

            Lfh = 0
            h = d - r + delta*nOtpsi[0,0]
            nOnpsi = nO.T@npsi

            Lgh = np.array([nOtpsi[0] + delta/d*(1-nOtpsi[0]**2), delta*nOnpsi[0]]).T
            LghLgh = Lgh@Lgh.T
            
            barrier_bits.append([h, Lfh, Lgh, LghLgh])

        return barrier_bits

