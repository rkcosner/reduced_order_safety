#!/usr/bin/env python
import rospkg
import sys 
rospack = rospkg.RosPack()
sys.path.append(rospack.get_path('red_ord_unitree')+'/src/utils/')

from unitree_safe_vel_utils import * 


if __name__ =="__main__": 

    # Get Experiment Type Param 
    experiment_type = rospy.get_param("/safe_velocity_node/experiment_type")
    if experiment_type == 0: 
        print("Running in simulation mode")
    elif experiment_type == 1: 
        print("Running in indoor mode")
    elif experiment_type == 2: 
        print("Running in outdoor mode")




    # Create node and loop publishing commands
    node = safe_velocity_node(experiment_type)
    rate = rospy.Rate(controller_freq) # 10hz
    while not rospy.is_shutdown():
        node.pubCmd()
        rate.sleep()


    # Save Data
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
