import numpy as np 
import matplotlib.pyplot as plt 
from os import listdir

from numpy.core.fromnumeric import squeeze


plt.rcParams["figure.figsize"] = (3.5,2.5)

DO = 0.5 + 0.32
realsense_offset = 0.26

data_files = listdir('/home/rkcosner/Documents/Research/RO_unitree/catkin_ws/src/reduced_order_safety_unitree/datalogs/') 
data_files.sort(reverse=True)
for title in data_files: 
    if title[0] =='2': 
        break 

date = title[0:17]
full_path = "/home/rkcosner/Documents/Research/RO_unitree/catkin_ws/src/reduced_order_safety_unitree/datalogs/"
print("showing data from ", date)
learning_params = np.load(full_path+date+"learning_params.npy")
x_traj = np.load(full_path+date+"x_traj.npy")
# x_mocap_traj = np.load(full_path+date+"x_mocap_traj.npy")
u_traj = np.load(full_path+date+"u_traj.npy")
u_des_traj = np.load(full_path+date+"u_des_traj.npy")
h_meas_traj = np.load(full_path+date+"h_meas_traj.npy")
h_true_traj = np.load(full_path+date+"h_true_traj.npy")
obs_traj = np.load(full_path+date+"obs_traj.npy")
tower1_mocap_pose = np.load(full_path+date+"tower1_mocap_pose.npy")
tower2_mocap_pose = np.load(full_path+date+"tower2_mocap_pose.npy")
tower3_mocap_pose = np.load(full_path+date+"tower3_mocap_pose.npy")


mocap_align_0 = np.array([realsense_offset, 0, 0, 0]) # zero themocap to align with 
# x_mocap_traj -= mocap_align_0
# tower1_mocap_pose -= mocap_align_0[0:2]
# tower2_mocap_pose -= mocap_align_0[0:2]
# tower3_mocap_pose -= mocap_align_0[0:2]

title = "learning params: [" + str(learning_params[0])+ ", " +str(learning_params[1]) +", "+str(learning_params[2]) +", "+str(learning_params[3]) +"]"

xO = [tower1_mocap_pose, tower2_mocap_pose, tower3_mocap_pose]

u_traj = np.squeeze(u_traj)
u_des_traj = np.squeeze(u_des_traj)

obs_traj = np.squeeze(np.array(obs_traj))
theta = np.linspace(0,2*np.pi + 0.1)
circ_x = DO*np.cos(theta)
circ_y = DO*np.sin(theta)


# Plot Everything

##### FIGURE 1 
plt.figure()
# Plot the state trajectory
plt.plot(x_traj[:,0],x_traj[:,1], 'b')
# Plot the measured obstacles
if len(obs_traj)>0:
    plt.plot(obs_traj[:,0], obs_traj[:,1], marker='.', color='y')
# Plot measured obstacle circles
for i, obs in enumerate(obs_traj): 
    if i < 20:
        for ob in obs.T:
            plt.plot(ob[0] + circ_x, ob[1] + circ_y, color='y', linestyle='--', linewidth=0.5)
# Plot the true obstacle circles
for xob in xO: 
    if xob.size > 0: 
        plt.plot(xob[0], xob[1], color = 'r', marker=".")
        plt.plot(xob[0] + circ_x, xob[1] + circ_y, 'r')
# Set axes properties and legend
plt.xlim([-1, 5])
plt.ylim([-4,2])
ax = plt.gca()
ax.set_aspect('equal')
# plt.legend(['state', 'obs1', 'obs2'])
plt.title(title)

plt.figure()
plt.plot(u_traj[:,2], u_traj[:,0:2], 'r--')
plt.plot(u_des_traj[:,2], u_des_traj[:,0:2], 'g')
plt.xlabel('time [s]')
plt.legend(['$v_{des}$', '$w_{des}$', '$v_{cbf}$', '$w_{cbf}$'])
plt.title(title)

plt.figure()
plt.plot(h_meas_traj[:,1], h_meas_traj[:,0], '--', linewidth=0.5)
plt.plot(h_true_traj[:,1], h_true_traj[:,0])
plt.xlabel("time [s]")
plt.hlines(0, xmin=0, xmax=h_meas_traj[-1,1])
plt.legend(['Measured CBF', 'True CBF'])
plt.title(title)

# ## ANIL: Added epsilon plot
# plt.figure()
# plt.plot(epsilon_traj)
# plt.legend(['epsilon'])

plt.show()