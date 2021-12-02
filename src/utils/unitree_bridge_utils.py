#!/usr/bin/env python
import rospy
import rospkg
import sys 
rospack = rospkg.RosPack()
sys.path.append(rospack.get_path('red_ord_unitree')+'/src/utils/')
from general import *

# Import message types
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


# Import unitree python interface
sys.path.append('/home/rkcosner/Documents/Research/Unitree/unitree_ws/src/mpac_a1/atnmy')
from mpac_cmd import * 

bounding_x = [-1, 5]
bounding_y = [-4, 2]

class unitree_bridge_node(): 
    def __init__(self, isSim):
        
        # Initialize the Node and Stand in Place
        stand_idqp()
        rospy.init_node('unitree_bridge', anonymous=True)
        self.rate = rospy.Rate(unitree_bridge_freq) # 10hz

        # Setup Publishers and Subscribers
        rospy.Subscriber("/cmd", Twist, self.velCallback)
        rospy.Subscriber("/simOnSwitch", Bool, self.simOnSwitchCallback)
        self.pubUnitreeStates = rospy.Publisher('/unitreeOnboardStates', Twist, queue_size=1)
        self.pubVis = rospy.Publisher('unitree_pose', Marker, queue_size=1 )

        # Object Attributes
        self.isSim = isSim
        self.running = isSim
        self.markerExists = False

    def simOnSwitchCallback(self, msg): 
        self.running = msg.data
        info = "Sim on/off switch changed to " + str(msg.data)
        rospy.loginfo(info)

    def read_and_publish_tlm(self):

        # Catch Error if there's no tlm yet because the unitree hasn't started
        tlm_data = get_tlm_data()
        if tlm_data == None: 
            # rospy.logerr("No tlm received from unitree")
            rospy.sleep(0.5)
            return 

        # Publish unitree onboard states as Twist
        msg = Twist()
        # Poses
        msg.linear.x = tlm_data[5][0] 
        msg.linear.y = tlm_data[5][1]
        msg.linear.z = tlm_data[5][5]
        # Velocities
        msg.angular.x = tlm_data[6][0] 
        msg.angular.y = tlm_data[6][1] 
        msg.angular.z = tlm_data[6][5]


        # Stop sim if outside bounding box
        dist_goal = np.linalg.norm(xgoal.T - np.array([msg.linear.x, msg.linear.y]).T)
        if self.isSim: 
            if msg.linear.x > bounding_x[1] or msg.linear.x <bounding_x[0] or msg.linear.y > bounding_y[1] or msg.linear.y <bounding_y[0] or dist_goal < 0.2: 
                self.running = False 
                print("Out of Bounding Box")

        if self.running: 
            self.pubUnitreeStates.publish(msg)


        # Create or Update Marker and publish
        pose = np.array([[msg.linear.x, msg.linear.y, msg.linear.z]]).T
        quadMarker = createMarker(4, pose, self.markerExists)
        self.pubVis.publish(quadMarker)
        if not self.markerExists: 
            self.markerExists = True

    def velCallback(self, data):
        # Send velocities to the unitree
        # Send lie command if the experiment is ended
        des_vx = data.linear.x
        des_wz = data.angular.z
        if self.running:
            walk_mpc_idqp(vx = des_vx, vrz = des_wz)
        else: 
            lie()
