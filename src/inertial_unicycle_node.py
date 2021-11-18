import rospy 
import numpy as np 
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import LinkStates
import std_msgs.msg
import matplotlib.pyplot as plt 
from datetime import datetime
import os 


class inertial_uni_node(): 

    def __init__(self): 

        ## Init ROS node
        rospy.init_node('inertial_uni_node', anonymous=True)

        ## Init publishers and subscribers: 
        #   /cmd                    :   sends velocity reference commands 
        #   /gazebo/link_states     :   gets position values from Cassie simulator 
        #   /kill_cmd               :   ends simulation if a command of pose.x>0      
        self.sub = rospy.Subscriber('cmd', Twist, self.cmdCallback)
        self.pub = rospy.Publisher('simStates', Twist, queue_size = 0 )

        ## Set Object Parameters
        #   full_state              :   store the full double integrator robot state
        #   input                   :   [a_x, a_y] accelerations in the x and y directions
        #   Kv                      :   velocity tracking proportional constant
        #   dt                      :   time constant for the dynamics
        #   A                       :   Euler integration A
        #   B                       :   Euler integration B
        self.state = np.array([[0.,0.,0.,0.,0.,0.]]).T
        self.input = np.array([[0.,0.]]).T
        self.Kv = 1
        self.dt = 0.1

    def cmdCallback(self, data): 
        self.v_ref = np.array([[data.linear.x, data.linear.y]]).T
        # self.lowLevelController()

    def lowLevelController(self): 
        self.input = -self.Kv*(self.state[] - self.v_ref)
        
    def updateState(self): 
        self.eulerStep()
        msg = Twist()
        msg.linear.x = self.state[0,0]
        msg.linear.y = self.state[1,0]
        msg.linear.z = np.arctan2(self.state[3,0], self.state[2,0]) - np.pi/2
        self.pub.publish(msg)

    def eulerStep(self):
        x = self.state 
        dyn = np.array([[
            x[3,0], 
            x[4,0],
            x[5,0],
            np.cos(x[2,0])*self.input[0,0], 
            np.sin(x[2,0])*self.input[0,0], 
            self.input[1,0] 
        ]])
        self.state = x + self.dt*dyn


if __name__ =="__main__": 

    # Init Node
    node = inertial_uni_node()
    rate = rospy.Rate(1/node.dt) # 1000hz
    
    # Run till a kill_cmd is received 
    while not rospy.is_shutdown():
        node.updateState()
        rate.sleep()