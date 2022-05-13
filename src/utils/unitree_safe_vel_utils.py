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
    #Changed to drive forward more
    udes = np.array([[Kv*np.linalg.norm(x-xgoal)+0.05, 
                    -Kom*(np.sin(psi) - (xgoal[1][0]-x[1][0])/np.linalg.norm(x-xgoal) )]]).T

    udes = saturate(udes)

    return udes
        

def saturate(input): 
    max_input = [[0.3, -0.2], [0.4, -0.4]]

    for i in range(2):
        if input[i]>max_input[i][0]:
            input[i]=max_input[i][0]
            # rospy.logerr("Input Saturation Hit ")
        elif input[i]<max_input[i][1]: 
            input[i]=max_input[i][1]   
            rospy.logerr("Input Saturation Hit ")
    return input




class safe_velocity_node(): 

    def __init__(self, experiment_type, A, B, C, D): 
        rospy.init_node('safe_velocity', anonymous=True)

        self.flag_state_received = False

        self.pub = rospy.Publisher('cmd', Twist, queue_size=1)

        # record values 
        self.t0 = getTimeNow()
        self.t = getTimeNow()
        self.xO = np.array([[10.0,10.0,10.0,10.0], [10.0,10.0,10.0,10.0]])
        self.u_traj = []
        self.x_traj = []
        self.x_mocap_traj = []
        self.u_des_traj = []
        self.h_meas_traj = []
        self.h_true_traj = []
        self.obs_traj = []
        self.obs_time = []
        self.tower1_mocap_pose = np.empty((0,0))
        self.tower2_mocap_pose = np.empty((0,0))        
        self.tower3_mocap_pose = np.empty((0,0))     
        if experiment_type == 0:
            self.tower1_mocap_pose = [xOsim[0][0],xOsim[1][0] ]
            self.tower2_mocap_pose = [xOsim[0][1],xOsim[1][1] ]
        
        self.alpha = A
        self.sigma = B
        self.MRCBF_add = C
        self.MRCBF_mult = D

        if experiment_type == 0: #Sim 
            # Use default barrier positions (in unitree_safe_vel_utils) and use true sim position
            par["xO"] = xOsim
            rospy.Subscriber('unitreeOnboardStates', Twist, self.stateCallback)
        elif experiment_type == 1: # In lab  
            # Use measured barrier positions and SLAM. Also, record ground truth  
            rospy.Subscriber("/vrpn_client_node/Unitree/pose", PoseStamped, self.unitreeMocapCallback)
            rospy.Subscriber("/vrpn_client_node/duckie1/pose", PoseStamped, self.tower1Callback)
            rospy.Subscriber("/vrpn_client_node/duckie2/pose", PoseStamped, self.tower2Callback)
            rospy.Subscriber("/vrpn_client_node/duckie3/pose", PoseStamped, self.tower3Callback)
            rospy.Subscriber("/vrpn_client_node/duckie4/pose", PoseStamped, self.tower4Callback)

            self.state_mocap = np.array([[0,0,0]]).T

        self.experiment_type = experiment_type
        

        self.state = np.array([[0.,0.,0.]]).T
        self.cmdVel = Twist()



    def getCurrentTime(self):
        return getTimeNow() - self.t0 

    # def stateCallback(self, data): 
    #     if self.flag_state_received == False: 
    #         self.flag_state_received = True

    #     self.state[0,0] = data.linear.x
    #     self.state[1,0] = data.linear.y
    #     self.state[2,0] = data.linear.z
        
    #     # data logging
    #     self.x_traj.append([data.linear.x, data.linear.y, data.linear.z, data.angular.x, data.angular.y, data.angular.z, self.getCurrentTime()])
        

    #     self.obs_traj.append(par['xO'])
    #     self.obs_time.append(self.getCurrentTime())


    # def slamCallback(self, data): 
    #     if self.flag_state_received == False: 
    #         self.flag_state_received = True
        

    #     q_z = data.pose.pose.orientation.z
    #     q_w = data.pose.pose.orientation.w
    #     yaw = np.arctan2(2*q_w*q_z, 1-2*q_z**2)

    #     t_old = self.t

    #     x_now = data.pose.pose.position.x - realsense_offset*np.cos(yaw) 
    #     y_now = data.pose.pose.position.y - realsense_offset*np.sin(yaw)
    #     w_now = yaw
    #     self.t = self.getCurrentTime()

    #     dt = self.t - t_old
    #     vx = (x_now - self.state[0,0])/dt
    #     vy = (y_now - self.state[1,0])/dt
    #     vw = (w_now - self.state[2,0])/dt 

    #     self.state[0,0] = x_now 
    #     self.state[1,0] = y_now 
    #     self.state[2,0] = w_now

    #     self.x_traj.append([self.state[0,0], self.state[1,0], self.state[2,0], vx, vy, vw, self.getCurrentTime()])


    # def barrierIDsCallback(self, data):
    #     ## Receive and Update Barrier Positions
    #     # - reads barrier_IDs message 
    #     # - resets xO to the measured positions
    #     # - sets Dob to be the appropriate length 
    #     # - records obstacle locations

    #     print("new barriers")
    #     xO = []
    #     for marker in data.markers: 
    #         pose = [marker.pose.position.x, marker.pose.position.y]
    #         xO.append(pose)
    #         self.obs_traj.append(pose)
    #     xO = np.array(xO).T
    #     if len(xO) == 0: 
    #         xO = np.empty((0,0))
    #     par["xO"] = xO
    #     par["DO"] = (0.5+robot_radius)
    #     print(xO)

    def tower1Callback(self, data):
        self.xO[0,0] = data.pose.position.x
        self.xO[1,0] = data.pose.position.y
        par["xO"] = self.xO

    def tower2Callback(self, data):
        self.xO[0,1] = data.pose.position.x
        self.xO[1,1] = data.pose.position.y
        par["xO"] = self.xO

    def tower3Callback(self, data):
        self.xO[0,2] = data.pose.position.x
        self.xO[1,2] = data.pose.position.y
        par["xO"] = self.xO

    def tower4Callback(self, data):
        self.xO[0,3] = data.pose.position.x
        self.xO[1,3] = data.pose.position.y
        par["xO"] = self.xO


    def unitreeMocapCallback(self, data): 
        if self.flag_state_received == False: 
            self.flag_state_received = True
        
        q_z = data.pose.orientation.z
        q_w = data.pose.orientation.w
        yaw = np.arctan2(2*q_w*q_z, 1-2*q_z**2)
        self.state = np.array([[data.pose.position.x, data.pose.position.y+optitrack_adjust_y, yaw]]).T
        self.x_mocap_traj.append([data.pose.position.x, data.pose.position.y+optitrack_adjust_y, yaw, self.getCurrentTime()])


    def pubCmd(self):
        if self.flag_state_received == False: 
            return 

        # Get Desired Input
        u_des = K_des(self.state)
        self.u_traj.append([u_des[0,0], u_des[1,0],  self.getCurrentTime()])

        # Filter through CBF SOCP
        barrier_bits = self.getBarrierBits()
        u_np = K_CBF_SOCP(barrier_bits,u_des, self.alpha, self.sigma, self.MRCBF_add, self.MRCBF_mult)

        # Publish v_cmd
        u_np = saturate(u_np)
        self.cmdVel.angular.z = u_np[1,0]

        if self.experiment_type == 0: 
            # Add drift experimental
            cmd_err = 0#0.05
            self.cmdVel.linear.x = u_np[0,0]+cmd_err
        else:        
            self.cmdVel.linear.x = u_np[0,0]


        self.pub.publish(self.cmdVel)

        # Log Data
        self.u_des_traj.append([u_np[0,0], u_np[1,0], self.getCurrentTime()] )
        self.measureCBF()

    def measureCBF(self): 
        # Measured CBF Value 
        z = self.state
        h = measureCBFutil(z, par["xO"])
        self.h_meas_traj.append([h, self.getCurrentTime()])

        # if self.experiment_type == 1 or self.experiment_type == 0 : 
        #     # If in lab with mocap record ground truth
        #     towers = [self.tower1_mocap_pose, self.tower2_mocap_pose, self.tower3_mocap_pose]
        #     xO_true = []
        #     for t in towers: 
        #         if len(t) > 0: 
        #             xO_true.append(t)
        #     xO_true = np.asarray(xO_true).T
        # if self.experiment_type == 1: 
        #     z = self.state_mocap
        #     self.h_true_traj.append([measureCBFutil(z, xO_true), self.getCurrentTime()])
        # elif self.experiment_type == 0: 
        #     z = self.state
        #     self.h_true_traj.append([measureCBFutil(z, xO_true), self.getCurrentTime()])

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

