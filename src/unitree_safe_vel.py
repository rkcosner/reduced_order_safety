#!/usr/bin/env python
import rospkg
import sys 
rospack = rospkg.RosPack()
sys.path.append(rospack.get_path('red_ord_unitree')+'/src/utils/')

from unitree_safe_vel_utils import * 


if __name__ =="__main__": 

    # Get Experiment Type Param 
    experiment_type = rospy.get_param("/safe_velocity_node/experiment_type")
    learningParam_A = rospy.get_param("/safe_velocity_node/A") # alpha: barrier function parameter
    learningParam_B = rospy.get_param("/safe_velocity_node/B") # sigma: ISSf user tunable parameter
    learningParam_C = rospy.get_param("/safe_velocity_node/C") # epsilon*(L_ah+L_lfh_L_lghlgh): Mrcbf param
    learningParam_D = rospy.get_param("/safe_velocity_node/D") # epsilon*L_lgh: Mrcbf param 

    if experiment_type == 0: 
        print("Running in simulation mode")
    elif experiment_type == 1: 
        print("Running in indoor mode")
    elif experiment_type == 2: 
        print("Running in outdoor mode")

    print("Learning Parameters = [", learningParam_A, learningParam_B, learningParam_C, learningParam_D,"]")
    rospy.sleep(2)

    # Create node and loop publishing commands
    node = safe_velocity_node(experiment_type, learningParam_A, learningParam_B, learningParam_C, learningParam_D)
    rate = rospy.Rate(controller_freq) # 10hz

    period = 100
    step = 0 
    while not rospy.is_shutdown():
        node.pubCmd()
        if experiment_type == 0: 
            #Mess with the barrier measurements here for the simulation
            step+=1
            par['xO']= xOsim - 0.1*np.array([[0,0],[1,1]])#0.1*np.array([[np.cos(step/period*2*np.pi), np.sin(step/period*2*np.pi)],
                                #        [-np.sin(step/period*2*np.pi), np.cos(step/period*2*np.pi)]])#np.random.normal(loc=0, scale=0.1, size=xOsim.shape )
            if step == period:
                step = 0 

        rate.sleep()


    # Save Data
    print(node.obs_traj)
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
    np.save(filename_string+"_learning_params.npy", np.array([learningParam_A, learningParam_B, learningParam_C, learningParam_D]))
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
