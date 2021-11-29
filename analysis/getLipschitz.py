import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D 
import sys
sys.path.append('/home/rkcosner/Documents/Research/RO_unitree/catkin_ws/src/reduced_order_safety_unitree/src/utils/')
from unitree_safe_vel_utils import*

epsilon = 0.1
node = safe_velocity_node(0, 2, 0.5, 0, 0)
a,b = node.getBarrierBits()

xs = np.arange(-1, 4,0.2)
ys = np.arange(-4, 1,0.2)
psis = np.arange(0, 2*np.pi, 0.2)


points1 = []
points2 = []

h1s = np.zeros((len(xs), len(ys), len(psis)))
Lfh1s = np.zeros((len(xs), len(ys), len(psis)))
Lgh1s1 = np.zeros((len(xs), len(ys), len(psis)))
Lgh1s2 = np.zeros((len(xs), len(ys), len(psis)))
Lgh2s = np.zeros((len(xs), len(ys), len(psis)))


for ix, x in enumerate(xs): 
    for iy, y in enumerate(ys): 
        for ip,psi in enumerate(psis): 
            state = np.array([[x,y,psi]]).T
            node.state = state
            bits = node.getBarrierBits()

            if bits[0][0] >= 0: 
                points1.append([x, y, psi, bits[0][0], bits[0][1], bits[0][2][0][0], bits[0][2][0][1], bits[0][3][0][0]])
                h1s[ix, iy, ip] = node.alpha*bits[0][0]
                Lfh1s[ix, iy, ip] = bits[0][1]
                Lgh1s1[ix,iy,ip] = bits[0][2][0][0]
                Lgh1s2[ix,iy,ip] = bits[0][2][0][1]
                Lgh2s[ix,iy,ip] =  bits[0][3][0][0]

            if bits[1][0] >= 0: 
                points2.append([x, y, psi, bits[1][0], bits[1][1], bits[1][2][0][0], bits[1][2][0][1], bits[1][3][0][0]])


points1 = np.array(points1)
points2 = np.array(points2)

# print(h1s)
print("L_ah = ", np.max(np.abs(np.gradient(h1s)))/0.2)
print("L_fh = ", np.max(np.abs(np.gradient(Lfh1s)))/0.2)
print("L_lgh1 = ", np.max(np.abs(np.gradient(Lgh1s1)))/0.2)
print("L_lgh2 = ", np.max(np.abs(np.gradient(Lgh1s2)))/0.2)
print("L_lgh = ", np.sqrt((np.abs(np.max(np.gradient(Lgh1s1)))/0.2)**2 + (np.max(np.abs(np.gradient(Lgh1s2)))/0.2)**2))
print("L_lgh**2 = ", np.max(np.abs(np.gradient(Lgh2s)))/0.2)


print("\n\n epsilon = ", epsilon )
print("C = ", epsilon*(np.max(np.abs(np.gradient(h1s)))/0.2 + np.max(np.abs(np.gradient(Lgh2s)))/0.2 +   np.max(np.abs(np.gradient(Lfh1s)))/0.2))
print("D = ", epsilon*np.sqrt((np.abs(np.max(np.gradient(Lgh1s1)))/0.2)**2 + (np.max(np.abs(np.gradient(Lgh1s2)))/0.2)**2))
# fig = plt.figure()
# ax = fig.add_subplot(projection='3d')
# ax.scatter(points1[:,0], points1[:,1], points1[:,2], marker='.')
# plt.show()