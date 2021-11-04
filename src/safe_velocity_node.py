import rospy 
import numpy as np 
import scipy as sp 
import ecos 
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import LinkStates
import std_msgs.msg
import matplotlib.pyplot as plt 
from datetime import datetime
import os 

## Problem Params
#   cmd_frequency   :   node publication frequency 
#   xgoal           :   goal state
#   robot_radius    :   radius of the robot around its center, used to expand the obstacle
#   xO              :   position of the obstacle centers
cmd_frequency = 10 
xgoal = np.array([[4,-1]]).T
robot_radius = 0.32
xO = np.array([[1.5,0],[3, -1.5]]).T
# derived params
dim = len(xgoal)
DO = (0.5+robot_radius)*np.ones((1,2))


## Controller Params
#   Kp              :   proportional gain for the desired controller
#   alpha           :   alpha parameter for the reduced order safety
scale = 0.1
Kp = 1*scale
alpha = 0.2

## ANIL: Tuneable ISSf params 
# epsilon(h) = epsilon_0 * pow(e, lambda_0 * h)
epsilon_0 = 20e4
lambda_0 = 5*0

# Dictionary of parameters for easy access in node object
par = {
    'xgoal' : xgoal, 
    'xO'    : xO, 
    'DO'    : DO, 
    'Kp'    : Kp, 
    'alpha' : alpha, 
    'dim'   : dim,
    'epsilon_0' : epsilon_0,
    'lambda_0'  : lambda_0
}


class safe_velocity_node(): 

    def __init__(self): 

        ## Init ROS node
        rospy.init_node('safe_velocity_node', anonymous=True)

        ## Init publishers and subscribers: 
        #   /cmd                    :   sends velocity reference commands 
        #   /gazebo/link_states     :   gets position values from Cassie simulator 
        #   /kill_cmd               :   ends simulation if a command of pose.x>0      
        self.pub = rospy.Publisher('cmd', Twist, queue_size=1)
        self.sub = rospy.Subscriber('gazebo/link_states', LinkStates, self.stateCallbackHardware)
        self.sub = rospy.Subscriber('simStates', Twist, self.stateCallbackSimultion)
        self.sub = rospy.Subscriber('kill_cmd', Twist, self.killCallback)

        ## Set Object Parameters
        #   flag_state_received     :   flag change to true when first state message is received. Handshake before sending v_ref
        #   state                   :   store robot state
        #   cmdVel                  :   commanded velocity, v_ref
        #   kill_cmd                :   changes to 1 when kill_cmd is received and stops sending velocities. Then saves data and plots
        self.flag_state_received = False
        self.state = np.array([[0.,0.,0.]]).T
        self.cmdVel = Twist()
        self.kill_cmd = False

        ## Init Data Recording
        #   u_traj      :   vector of filtered v_ref through time
        #   x_trah      :   vector of states through time
        #   u_des_traj  :   vector of unfiltered velocities through time
        #   h_trah      :   vector of barrier values through time
        self.u_traj = []
        self.x_traj = []
        self.u_des_traj = []
        self.h_traj = []

        ## TISSf Init Data Recording
        #   epsilon_traj:   vector of variable T-ISSf modifications through time (hDot \geq -alpha(h) + ||Lgh||^2/epsilon(h) )
        self.epsilon_traj = []
        

        ## SOCP Params
        self.L_lgh = 0 
        self.L_lfh = 0
        self.L_ah = 0 
        self.gamma = 0 
        self.SOCP_dims = {
            'l':0,      # linear cone size
            'q':[4,3,3],  # second order cone size
            'e':0       # exponential cone sizes
        }

    def stateCallbackHardware(self, data): 
        ## Hardware State Reader
        # - Reads state message 
        # - Sets falg_state_received handshake 
        # - Sets self.state data 
        # - Records state trajectory data (x_traj)
        if self.flag_state_received == False: 
            self.flag_state_received = True

        self.state[0,0] = data.pose[6].position.x
        self.state[1,0] = data.pose[6].position.y
        self.state[2,0] = data.pose[6].orientation.z
        self.x_traj.append([data.pose[6].position.x, data.pose[6].position.y, data.pose[6].orientation.z])

    def stateCallbackSimultion(self, data): 
        ## Simulation State Reader
        # - Reads state message 
        # - Sets falg_state_received handshake 
        # - Sets self.state data 
        # - Records state trajectory data (x_traj)
        if self.flag_state_received: 
            self.flag_state_received = True

        self.state[0,0] = data.linear.x
        self.state[1,0] = data.linear.y
        self.state[2,0] = data.angular.z
        self.x_traj.append([data.linear.x, data.linear.y, data.angular.z])

    def killCallback(self, data): 
        # Kills the controller whenever a kill_cmd is published
        self.kill_cmd = True

    def pubCmd(self):
        ## Sends velocity command (v_ref)
        # - if no handshake, skip
        # - get filtered velocity command
        # - publish and record velocity command

        if self.flag_state_received: 
            return 
        ## ANIL: Added epsilon signal as an output
        # u_np , epslon = self.K_CBF()
        epslon = 0 
        u_socp = self.K_CBF_SOCP()
        u_np = u_socp

        self.cmdVel.linear.x = u_np[0,0]
        self.cmdVel.linear.y = u_np[1,0]

        # Publish for simple sim
        self.pub.publish(self.cmdVel)

        # Set Parameters for Cassie (cassie listens to these vvv rosparams instead of a velocity topic)
        rospy.set_param('cassie/locomotion_control/HLIP/HLIP_vxd', float(u_np[0,0]))
        rospy.set_param('cassie/locomotion_control/HLIP/HLIP_vyd', float(u_np[1,0]))

        # Data logging
        self.u_des_traj.append(u_np[0:2])
        self.h_traj.append(self.CBF()[0])
        self.epsilon_traj.append(epslon)

    def K_CBF(self):
        ## Get safe, filtered velocity command 
        z = self.state 
        alpha = par['alpha']
        udes =self.K_des()
        h, Lfh, Lgh, LghLgh = self.CBF() 
        # ANIL: Modified phi function with ISSf
        epslon = par['epsilon_0']*pow(2.71828,par['lambda_0']*h)
        phi = Lfh + Lgh@udes + alpha*h - 1/epslon
        u = udes + max(0, -phi)*Lgh.T/LghLgh
        # ANIL: send epsilon signal as an output
        return u, epslon

    def CBF(self): 
        ## Returns CBF values: h, Lfh, Lghm LghLgh 
        z = self.state
        dim = par['dim']
        xO = par['xO']
        DO = par['DO']
        x = z[0:dim,:]
        psi = z[dim,:][0]

        # Calculate h values
        hk = np.zeros((len(DO[0]),1))
        for kob in range(len(DO[0])): 
            xob = np.array([xO[:,kob]]).T
            Dob = DO[0,kob]
            hk[kob] = np.linalg.norm(x-xob) - Dob

        # Calculate Lfh, Lgh, LghLgh' for only the closest barrier (See: Egerstedt "Nonsmooth Composable CBFs")
        h = hk.min()
        idx = hk.argmin()
        xob = np.array([xO[:,idx]]).T
        gradh = ((x - xob)/np.linalg.norm(x - xob)).T 
        Lfh = 0 
        Lgh = gradh
        LghLgh = 1

        return h, Lfh, Lgh, LghLgh 


    def CBF1(self): 
        ## Returns CBF values: h, Lfh, Lghm LghLgh 
        z = self.state
        dim = par['dim']
        xO = par['xO']
        DO = par['DO']
        x = z[0:dim,:]
        psi = z[dim,:][0]

        # Calculate h values
        hk = np.zeros((len(DO[0]),1))

        kob = 0 
        xob = np.array([xO[:,kob]]).T
        Dob = DO[0,kob]
        h = np.linalg.norm(x-xob) - Dob


        # Calculate Lfh, Lgh, LghLgh' for only the closest barrier (See: Egerstedt "Nonsmooth Composable CBFs")
        idx = kob
        xob = np.array([xO[:,idx]]).T
        gradh = ((x - xob)/np.linalg.norm(x - xob)).T 
        Lfh = 0 
        Lgh = gradh
        LghLgh = 1

        return h, Lfh, Lgh, LghLgh 

    def CBF2(self): 
        ## Returns CBF values: h, Lfh, Lghm LghLgh 
        z = self.state
        dim = par['dim']
        xO = par['xO']
        DO = par['DO']
        x = z[0:dim,:]
        psi = z[dim,:][0]

        # Calculate h values
        hk = np.zeros((len(DO[0]),1))

        kob = 1 
        xob = np.array([xO[:,kob]]).T
        Dob = DO[0,kob]
        h = np.linalg.norm(x-xob) - Dob


        # Calculate Lfh, Lgh, LghLgh' for only the closest barrier (See: Egerstedt "Nonsmooth Composable CBFs")
        idx = kob
        xob = np.array([xO[:,idx]]).T
        gradh = ((x - xob)/np.linalg.norm(x - xob)).T 
        Lfh = 0 
        Lgh = gradh
        LghLgh = 1

        return h, Lfh, Lgh, LghLgh 

    def K_des(self):
        ## Records and returns unfiltered velocity values        
        z = self.state 
        xgoal = par['xgoal']
        dim = par['dim']
        x = z[0:dim,:]
        udes = -Kp*(x-xgoal)
        self.u_traj.append(udes)

        return udes


    def K_CBF_SOCP(self): 
        h1, Lfh1, Lgh1, LghLgh1 = self.CBF1()
        h2, Lfh2, Lgh2, LghLgh2 = self.CBF2()

        G = sp.sparse.csc_matrix(
            [[-1/np.sqrt(2), 0, 0], 
            [-1/np.sqrt(2), 0, 0 ], 
            [0, -1, 0], 
            [0, 0, -1], 
            [0, -Lgh1[0,0],  -Lgh1[0,1]],
            [0, -self.gamma*self.L_lgh, 0], 
            [0, 0, -self.gamma*self.L_lgh],
            [0, -Lgh2[0,0],  -Lgh2[0,1]],
            [0, -self.gamma*self.L_lgh, 0], 
            [0, 0, -self.gamma*self.L_lgh]]
        )
        b  = np.array(
            [1/np.sqrt(2), 
            -1/np.sqrt(2), 
            0, 
            0, 
            Lfh1 + par['alpha']*h1 - (self.L_lfh + self.L_ah)*self.gamma, 
            0, 
            0,
            Lfh2 + par['alpha']*h2 - (self.L_lfh + self.L_ah)*self.gamma, 
            0, 
            0]
        )
        u_des = self.K_des()
        cost = np.array([1.0, -u_des[0].item(),  -u_des[1].item()])
        ecos_solver_output = ecos.solve(cost, G, b, self.SOCP_dims, verbose=False)

        if ecos_solver_output['info']['exitFlag'] ==0 or ecos_solver_output['info']['exitFlag'] ==10: # ECOS Solver Successful
            u_des = np.expand_dims(ecos_solver_output['x'][1:3],1)
            return u_des
        else: # ECOS Solver Failed 
            rospy.logwarn('SOCP failed') # Filter falls back to previous self.inputAct_
            return np.array([[0],[0]])


if __name__ =="__main__": 

    # Init Node
    node = safe_velocity_node()
    rate = rospy.Rate(cmd_frequency) # 10hz
    
    # Run till a kill_cmd is received 
    while not rospy.is_shutdown():
        node.pubCmd()
        if node.kill_cmd: 
            break
        rate.sleep()



    # Save Data and Plot
    print("Saving_data")

    x_traj = np.squeeze(np.array(node.x_traj))
    u_traj = np.squeeze(np.array(node.u_traj))
    u_des_traj = np.squeeze(np.array(node.u_des_traj))
    h_traj = np.array(node.h_traj)
    # ANIL: Added epsilon array
    epsilon_traj = np.array(node.epsilon_traj)

    today = datetime.now()
    print(os.getcwd())
    filename_string = "/home/drew/catkin_ws_cassie/src/red_ord_cassie/datalogs/" + today.strftime("%Y_%m_%d_%H_%M")
    print(filename_string)
    np.save(filename_string+"_x_traj.npy", node.x_traj)
    np.save(filename_string+"_u_traj.npy", node.u_traj)
    np.save(filename_string+"_u_des_traj.npy", node.u_des_traj)
    np.save(filename_string+"_h_traj.npy", node.h_traj)
    np.save(filename_string+"_eps_traj.npy", node.epsilon_traj)


    # Obstacle Details for plotting
    theta = np.linspace(0,2*np.pi + 0.1)
    circ_x = DO[0,0]*np.cos(theta)
    circ_y = DO[0,0]*np.sin(theta)


    # Plot Everything
    plt.figure()
    plt.plot(x_traj[:,0],x_traj[:,1])
    for xob in xO.T: 
        plt.plot(xob[0], xob[1], 'r')
        plt.plot(xob[0] + circ_x, xob[1] + circ_y, 'r')
    ax = plt.gca()
    ax.set_aspect('equal')
    plt.legend(['state', 'obs1', 'obs2'])

    plt.figure()
    plt.plot(u_traj, 'r--')
    plt.plot(u_des_traj, 'g')
    plt.legend(['$v_{des}$', '$w_{des}$', '$v_{cbf}$', '$w_{cbf}$'])

    plt.figure()
    plt.plot(h_traj)
    plt.hlines(0, xmin=0, xmax=len(h_traj), linestyles='dashed')
    plt.legend(['CBF'])

    ## ANIL: Added epsilon plot
    plt.figure()
    plt.plot(epsilon_traj)
    plt.legend(['epsilon'])

    plt.show()
