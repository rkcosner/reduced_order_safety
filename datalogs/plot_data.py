import numpy as np 
import matplotlib.pyplot as plt 
from os import listdir

from numpy.core.fromnumeric import squeeze

DO = 0.5


data_files = listdir('./') 
data_files.sort(reverse=True)
for title in data_files: 
    if title[0] =='2': 
        break 

date = title[0:17]
print("showing data from ", date)
learning_params = np.load("./"+date+"learning_params.npy")
x_traj = np.load("./"+date+"x_traj.npy")
x_mocap_traj = np.load("./"+date+"x_mocap_traj.npy")
u_traj = np.load("./"+date+"u_traj.npy")
u_des_traj = np.load("./"+date+"u_des_traj.npy")
h_meas_traj = np.load("./"+date+"h_meas_traj.npy")
h_true_traj = np.load("./"+date+"h_true_traj.npy")
obs_traj = np.load("./"+date+"obs_traj.npy")
tower1_mocap_pose = np.load("./"+date+"tower1_mocap_pose.npy")
tower2_mocap_pose = np.load("./"+date+"tower2_mocap_pose.npy")
tower3_mocap_pose = np.load("./"+date+"tower3_mocap_pose.npy")

print(obs_traj)
title = "learning params: [" + str(learning_params[0])+ ", " +str(learning_params[1]) +", "+str(learning_params[2]) +", "+str(learning_params[3]) +"]"

xO = [tower1_mocap_pose, tower2_mocap_pose, tower3_mocap_pose]

u_traj = np.squeeze(u_traj)
u_des_traj = np.squeeze(u_des_traj)

obs_traj = np.squeeze(obs_traj)
theta = np.linspace(0,2*np.pi + 0.1)
circ_x = DO*np.cos(theta)
circ_y = DO*np.sin(theta)

# Plot Everything
plt.figure()
plt.plot(x_traj[:,0],x_traj[:,1])
if len(x_mocap_traj)>0:
    plt.plot(x_mocap_traj[:,0], x_mocap_traj[:,1])# epsilon*(L_ah+L_lfh_L_lghlgh): Mrcbf param
for i, obs in enumerate(obs_traj):
    plt.plot(obs[0,:], obs[1,:],  'y.')
    for ob in obs.T: 
        if i < 20:
            plt.plot(ob[0] + circ_x, ob[1] + circ_y, color='y', linestyle='--', linewidth=0.5)

for xob in xO: 
    if xob.size > 0: 
        plt.plot(xob[0], xob[1], 'r')
        plt.plot(xob[0] + circ_x, xob[1] + circ_y, 'r')
ax = plt.gca()
ax.set_aspect('equal')
plt.legend(['state', 'obs1', 'obs2'])
plt.title(title)

plt.figure()
plt.plot(u_traj, 'r--')
plt.plot(u_des_traj, 'g')
plt.legend(['$v_{des}$', '$w_{des}$', '$v_{cbf}$', '$w_{cbf}$'])
plt.title(title)

plt.figure()
plt.plot(h_meas_traj, '--', linewidth=0.5)
plt.plot(h_true_traj)
plt.hlines(0, xmin=0, xmax=len(h_meas_traj))
plt.legend(['Measured CBF', 'True CBF'])
plt.title(title)

# ## ANIL: Added epsilon plot
# plt.figure()
# plt.plot(epsilon_traj)
# plt.legend(['epsilon'])

plt.show()