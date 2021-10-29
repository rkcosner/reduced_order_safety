import rospy 
import numpy as np 
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
#   x0              :   initial state
cmd_frequency = 10 
xgoal = np.array([[4,-1]]).T
robot_radius = 0.32
xO = np.array([[1.5,0],[3, -1.5]]).T
# derived params
dim = len(xgoal)
DO = (0.5+robot_radius)*np.ones((1,2))


## Controller Params
#   Kp              :   proportional gain for the desired controller
#
#
scale = 0.1
Kp = 1*scale
alpha = 0.2
Kv = 0.08*scale


# Store Parameters in Dict
par = {
    'xgoal' : xgoal, 
    'xO'    : xO, 
    'DO'    : DO, 
    'Kp'    : Kp, 
    'alpha' : alpha, 
    'Kv'    : Kv, 
    'dim'   : dim,
    # 'delta' : delta,
    # 'Kom'   : Kom,
    # 'R'     : R
}

class safe_velocity_node(): 

    def __init__(self): 
        self.pub = rospy.Publisher('cmd', Twist, queue_size=1)
        self.sub = rospy.Subscriber('gazebo/link_states', LinkStates, self.stateCallback)
        self.sub = rospy.Subscriber('kill_cmd', Twist, self.killCallback)


        rospy.init_node('safe_velocity_node', anonymous=True)

        self.flag_state_received = False
        self.state = np.array([[0.,0.,0.]]).T
        self.cmdVel = Twist()
        self.kill_cmd = 0
        # self.cmdVel.status = 1

        # record values 
        self.u_traj = []
        self.x_traj = []
        self.u_des_traj = []
        self.h_traj = []


    def stateCallback(self, data): 
        if self.flag_state_received == False: 
            self.flag_state_received = True

        self.state[0,0] = data.pose[6].position.x
        self.state[1,0] = data.pose[6].position.y
        self.state[2,0] = data.pose[6].orientation.z
        # data logging
        # print(self.state)
        # print()
        self.x_traj.append([data.pose[6].position.x, data.pose[6].position.y, data.pose[6].orientation.z])

    def killCallback(self, data): 
        if data.linear.x > 1: 
            self.kill_cmd = 1

    def pubCmd(self):
        if self.flag_state_received == False: 
            return 

        # self.cmdVel.header = std_msgs.msg.Header()

        u_np = self.K_CBF()

        self.cmdVel.linear.x = u_np[0,0]
        self.cmdVel.linear.y = u_np[1,0]
        # self.cmdVel.cmd = [u_np[0,0], u_np[1,0]]
        self.pub.publish(self.cmdVel)

        # data logging
        self.u_des_traj.append(u_np[0:2])
        self.h_traj.append(self.CBF()[0])
        print("setting  params")
        rospy.set_param('cassie/locomotion_control/HLIP/HLIP_vxd', float(u_np[0,0]))
        rospy.set_param('cassie/locomotion_control/HLIP/HLIP_vyd', float(u_np[1,0]))

    def K_CBF(self):
        z = self.state 
        alpha = par['alpha']
        # R = par['R']
        udes =self.K_des()
        #Safety Filter 
        h, Lfh, Lgh, LghLgh = self.CBF() 
        # W=np.array([[1,0],[0,1/R]])
        phi = Lfh + Lgh@udes + alpha*h
        u = udes + max(0, -phi)*Lgh.T/LghLgh
        # u = udes + W@(max(0,-phi)*(Lgh@W).T)/((Lgh@W)@(Lgh@W).T)

        return u



    # Composed CBF Value 
    def CBF(self): 
        z = self.state
        dim = par['dim']
        xO = par['xO']
        DO = par['DO']
        # delta = par['delta']

        x = z[0:dim,:]
        psi = z[dim,:][0]

        tpsi = np.array([[np.cos(psi), np.sin(psi)]]).T
        npsi = np.array([[-np.sin(psi),np.cos(psi)]]).T

        # Control Barrier Function 
        hk = np.zeros((len(DO[0]),1))
        for kob in range(len(DO[0])): 
            xob = np.array([xO[:,kob]]).T
            Dob = DO[0,kob]
            hk[kob] = np.linalg.norm(x-xob) - Dob
            # xob = np.array([xO[:,kob]]).T
            # robs = DO[0,kob]
            # dobs = np.linalg.norm(x-xob)

            # nobs = (x - xob)/dobs
            # nobstpsi = nobs.T@tpsi
            # hk[kob] = dobs - robs +delta*nobstpsi
        # Use only the closest obstacle
        h = hk.min()
        idx = hk.argmin()
        xob = np.array([xO[:,idx]]).T
        gradh = ((x - xob)/np.linalg.norm(x - xob)).T 
        Lfh = 0 
        Lgh = gradh
        LghLgh = 1
        # d = np.linalg.norm(x - xob)
        # r = DO[0,idx]
        # nO = (x - xob)/d 
        # nOtpsi = nO.T@tpsi #np.dot(nO.T, tpsi)


        # Lfh = 0
        # h = d - r + delta*nOtpsi[0,0]
        # nOnpsi = nO.T@npsi

        # Lgh = np.array([nOtpsi[0] + delta/d*(1-nOtpsi[0]**2), delta*nOnpsi[0]]).T
        # LghLgh = Lgh@Lgh.T


        return h, Lfh, Lgh, LghLgh 


    def K_des(self):
        z = self.state 
        xgoal = par['xgoal']
        dim = par['dim']
        # Kom = par['Kom']
        x = z[0:dim,:]
        udes = -Kp*(x-xgoal)
        # psi = z[dim,:][0]
        # udes = np.array([[Kv*np.linalg.norm(x-xgoal), 
        #                 -Kom*(np.sin(psi) - (xgoal[1][0]-x[1][0])/np.linalg.norm(x-xgoal) )]]).T
        # data logging
        self.u_traj.append(udes)

        return udes
        
        



if __name__ =="__main__": 
    node = safe_velocity_node()
    rate = rospy.Rate(cmd_frequency) # 10hz
    while not rospy.is_shutdown():
        node.pubCmd()
        if node.kill_cmd>=1: 
            break
        rate.sleep()

    print("saving_data")

    x_traj = np.squeeze(np.array(node.x_traj))
    u_traj = np.squeeze(np.array(node.u_traj))
    u_des_traj = np.squeeze(np.array(node.u_des_traj))

    print(x_traj)

    today = datetime.now()

    print(os.getcwd())

    date_string = "/home/drew/ReducedOrderSafety/catkin_ws/src/mrcbf_IROS21/datalogs/" + today.strftime("%Y_%m_%d_%H_%M")
    print(date_string)
    np.save(date_string+"_x_traj.npy", node.x_traj)
    np.save(date_string+"_u_traj.npy", node.u_traj)
    np.save(date_string+"_u_des_traj.npy", node.u_des_traj)
    np.save(date_string+"_h_traj.npy", node.h_traj)
    

    h_traj = np.array(node.h_traj)

    theta = np.linspace(0,2*np.pi + 0.1)
    circ_x = DO[0,0]*np.cos(theta)
    circ_y = DO[0,0]*np.sin(theta)

    plt.figure()

    print("x  = ", x_traj[:,0])
    print("y  = ", x_traj[:,1])

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

    plt.show()
