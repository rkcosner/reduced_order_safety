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


class unitree_bridge_node(): 
    def __init__(self):
        
        # Start Standing 
        stand_idqp()

        # Initialize the Node
        rospy.init_node('unitree_bridge', anonymous=True)
        self.rate = rospy.Rate(unitree_bridge_freq) # 10hz

        # Setup Publishers and Subscribers
        rospy.Subscriber("/cmd", Twist, self.velCallback)
        rospy.Subscriber("/simOnSwitch", Bool, self.simOnSwitchCallback)
        self.pub = rospy.Publisher('/simStates', Twist, queue_size=1)

        # Object Attributes
        self.running = False

    def simOnSwitchCallback(self, msg): 
        self.running = msg.data

    def read_and_publish_tlm(self):
        tlm_data = get_tlm_data()
        # if tlm_data.size>0:
        #     print(tlm_data[5])
        # else: 
        #     print("empty") 
        # rospy.loginfo(hello_str)
        # pub.publish(hello_str)

        msg = Twist()
        msg.linear.x = tlm_data[5][0] 
        msg.linear.y = tlm_data[5][1]
        msg.linear.z = tlm_data[5][5]
        # msg.angular.z = np.sin(tlm_data[5][5]/2)
        # msg.angular.w = np.cos(tlm_data[5][5]/2)
        
        self.pub.publish(msg)


    def velCallback(self, data):
        des_vx = data.linear.x
        des_wz = data.angular.z
        if self.running:
            walk_mpc_idqp(vx = des_vx, vrz = des_wz)
        else: 
            lie()
