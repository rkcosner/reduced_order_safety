#!/usr/bin/env python
import rospkg
import sys 
rospack = rospkg.RosPack()
sys.path.append(rospack.get_path('red_ord_unitree')+'/src/utils/')

from unitree_safe_vel_utils import * 



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

class safe_velocity_node(): 

    def __init__(self): 
        rospy.init_node('safe_velocity', anonymous=True)

        self.flag_state_received = False

        self.pub = rospy.Publisher('cmd', Twist, queue_size=1)

        # if True: # use unitree onboard states         
        rospy.Subscriber('unitreeOnboardStates', Twist, self.stateCallback)
        # else: # Use SLAM
        # rospy.Subscriber('t265/odom/sample', Odometry, self.slamCallback) 

        rospy.Subscriber('barrier_IDs', MarkerArray, self.barrierIDsCallback)


        self.state = np.array([[0.,0.,0.]]).T
        self.cmdVel = Twist()

        # record values 
        self.u_traj = []
        self.x_traj = []
        self.u_des_traj = []
        self.h_traj = []
        self.obs_traj = []

        self.L_ah = 1 
        self.L_lfh = 1 
        self.L_lgh = 1 
        self.L_lgh2 = 1 
        self.sigma = 0.0
        self.gamma = 0.0

        # Visualization 
        self.quadMarker = Marker()


    def stateCallback(self, data): 
        if self.flag_state_received == False: 
            self.flag_state_received = True

        self.state[0,0] = data.linear.x
        self.state[1,0] = data.linear.y
        self.state[2,0] = data.angular.z
        # data logging

        point = np.array([[data.linear.x, data.linear.y, data.linear.z]]).T
        self.x_traj.append([data.linear.x, data.linear.y, data.linear.z])


    def slamCallback(self, data): 
        if self.flag_state_received == False: 
            self.flag_state_received = True
        

        q_z = data.pose.pose.orientation.z
        q_w = data.pose.pose.orientation.w
        yaw = np.arctan2(2*q_w*q_z, 1-2*q_z**2)

        self.state[0,0] = data.pose.pose.position.x - visual_offset*np.cos(yaw)
        self.state[1,0] = data.pose.pose.position.y - visual_offset*np.sin(yaw) 
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
        par["xO"] = xO
        par["DO"] = (0.5+robot_radius)
        print(xO)


    def pubCmd(self):
        if self.flag_state_received == False: 
            return 


        u_np = self.K_CBF_SOCP()#self.K_CBF()
        # print("SOCP", u_np)
        # u_np = self.K_CBF()
        # print("HAND", u_np)

        self.cmdVel.linear.x = u_np[0,0]
        self.cmdVel.angular.z =  u_np[1,0]
        self.pub.publish(self.cmdVel)

        # data logging
        self.u_des_traj.append(u_np[0:2])


    def K_CBF_SOCP(self):
        barrier_bits = self.getBarrierBits()
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
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       
        u_des = self.K_des()
        cost = np.array([1.0, -u_des[0].item(),  R**2*-u_des[1].item()])
        ecos_solver_output = ecos.solve(cost, G, b, SOCP_dims, verbose=False)

        if ecos_solver_output['info']['exitFlag'] ==0 or ecos_solver_output['info']['exitFlag'] ==10: # ECOS Solver Successful
            u_des = np.expand_dims(ecos_solver_output['x'][1:3],1)
            return u_des
        else: # ECOS Solver Failed 
            rospy.logwarn('SOCP failed') # Filter falls back to previous self.inputAct_
            return np.array([[0],[0]])


    # Composed CBF Value 
    def CBF(self): 
        z = self.state
        dim = par['dim']
        xO = par['xO']
        DO = par['DO']
        delta = par['delta']

        x = z[0:dim,:]
        psi = z[dim,:][0]

        tpsi = np.array([[np.cos(psi), np.sin(psi)]]).T
        npsi = np.array([[-np.sin(psi),np.cos(psi)]]).T

        # Control Barrier Function 
        hk = np.zeros((xO.shape[1],1))
        for kob in range(xO.shape[1]): 
            xob = np.array([xO[:,kob]]).T
            robs = DO[0,kob]
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
            self.h_traj.append(h)
        else: 
            self.h_traj.append(0.42)



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

    def K_des(self):
        z = self.state 
        xgoal = par['xgoal']
        dim = par['dim']
        Kv = par['Kv']    
        Kom = par['Kom']
        x = z[0:dim,:]
        psi = z[dim,:][0]
        udes = np.array([[Kv*np.linalg.norm(x-xgoal), 
                        -Kom*(np.sin(psi) - (xgoal[1][0]-x[1][0])/np.linalg.norm(x-xgoal) )]]).T
        # data logging
        self.u_traj.append(udes)

        return udes
        


if __name__ =="__main__": 
    node = safe_velocity_node()
    rate = rospy.Rate(controller_freq) # 10hz
    while not rospy.is_shutdown():
        node.pubCmd()

        rate.sleep()

    x_traj = np.squeeze(np.array(node.x_traj))
    u_traj = np.squeeze(np.array(node.u_traj))
    u_des_traj = np.squeeze(np.array(node.u_des_traj))
    obs_traj = np.squeeze(np.array(node.obs_traj))
    h_traj = np.array(node.h_traj)

    print(x_traj)

    today = datetime.now()

    print(os.getcwd())

    filename_string = "/home/rkcosner/Documents/Research/RO_unitree/catkin_ws/src/reduced_order_safety_unitree/datalogs/" + today.strftime("%Y_%m_%d_%H_%M")
    print(filename_string)
    np.save(filename_string+"_x_traj.npy", node.x_traj)
    np.save(filename_string+"_u_traj.npy", node.u_traj)
    np.save(filename_string+"_u_des_traj.npy", node.u_des_traj)
    np.save(filename_string+"_h_traj.npy", node.h_traj)
    np.save(filename_string+"_obs_traj.npy", node.obs_traj)
