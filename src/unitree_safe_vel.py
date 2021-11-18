#!/usr/bin/env python
import rospkg
import sys 
rospack = rospkg.RosPack()
sys.path.append(rospack.get_path('red_ord_unitree')+'/src/utils/')

from unitree_safe_vel_utils import * 



class safe_velocity_node(): 

    def __init__(self, experiment_type): 
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
        
        self.L_ah = 1 
        self.L_lfh = 1 
        self.L_lgh = 1 
        self.L_lgh2 = 1 
        self.sigma = 0.0
        self.gamma = 0.0
        self.alpha =  10


    def stateCallback(self, data): 
        if self.flag_state_received == False: 
            self.flag_state_received = True

        self.state[0,0] = data.linear.x
        self.state[1,0] = data.linear.y
        self.state[2,0] = data.angular.z
        
        # data logging
        self.x_traj.append([data.linear.x, data.linear.y, data.linear.z])


    def slamCallback(self, data): 
        if self.flag_state_received == False: 
            self.flag_state_received = True
        

        q_z = data.pose.pose.orientation.z
        q_w = data.pose.pose.orientation.w
        yaw = np.arctan2(2*q_w*q_z, 1-2*q_z**2)

        self.state[0,0] = data.pose.pose.position.x - realsense_offset*np.cos(yaw)
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

        u_np = K_CBF_SOCP(barrier_bits, u_des, self.L_ah, self.L_lfh, self.L_lgh, self.L_lgh2, self.sigma, self.gamma, self.alpha)
        u_np = saturate(u_np)

        # Publish v_cmd
        self.cmdVel.linear.x = u_np[0,0]
        self.cmdVel.angular.z =  u_np[1,0]
        self.pub.publish(self.cmdVel)

        # Log Data
        self.u_des_traj.append(u_np[0:2])
        self.measureCBF()

    def measureCBF(self): 
        # Measured CBF Value 
        z = self.state
        h = measureCBFutil(z, par["xO"])
        self.h_meas_traj.append(h)

        if self.experiment_type == 1: 
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




if __name__ =="__main__": 

    experiment_type = rospy.get_param("/safe_velocity_node/experiment_type")

    if experiment_type == 0: 
        print("Running in simulation mode")
    elif experiment_type == 1: 
        print("Running in indoor mode")
    elif experiment_type == 2: 
        print("Running in outdoor mode")


    node = safe_velocity_node(experiment_type)
    rate = rospy.Rate(controller_freq) # 10hz
    while not rospy.is_shutdown():
        node.pubCmd()

        rate.sleep()

    x_traj = np.squeeze(np.array(node.x_traj))
    x_mocap_traj = np.squeeze(np.array(node.x_mocap_traj))
    u_traj = np.squeeze(np.array(node.u_traj))
    u_des_traj = np.squeeze(np.array(node.u_des_traj))
    obs_traj = np.squeeze(np.array(node.obs_traj))
    h_meas_traj = np.array(node.h_meas_traj)
    h_true_traj = np.array(node.h_true_traj)


    today = datetime.now()


    filename_string = "/home/rkcosner/Documents/Research/RO_unitree/catkin_ws/src/reduced_order_safety_unitree/datalogs/" + today.strftime("%Y_%m_%d_%H_%M")
    print(filename_string)
    np.save(filename_string+"_x_traj.npy", node.x_traj)
    np.save(filename_string+"_x_mocap_traj.npy", node.x_mocap_traj)
    np.save(filename_string+"_u_traj.npy", node.u_traj)
    np.save(filename_string+"_u_des_traj.npy", node.u_des_traj)
    np.save(filename_string+"_h_meas_traj.npy", node.h_meas_traj)
    np.save(filename_string+"_h_true_traj.npy", node.h_true_traj)
    np.save(filename_string+"_obs_traj.npy", node.obs_traj)
    np.save(filename_string+"_tower1_mocap_pose.npy", node.tower1_mocap_pose)
    np.save(filename_string+"_tower2_mocap_pose.npy", node.tower2_mocap_pose)
    np.save(filename_string+"_tower3_mocap_pose.npy", node.tower3_mocap_pose)
