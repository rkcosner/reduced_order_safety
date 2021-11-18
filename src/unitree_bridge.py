#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import sys 
sys.path.append('/home/rkcosner/Documents/Research/Unitree/unitree_ws/src/mpac_a1/atnmy')
from mpac_cmd import * 

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist


class node(): 
    def __init__(self):
        stand_idqp()
        rospy.init_node('unitree_tlm', anonymous=True)
        
        self.rate = rospy.Rate(5) # 10hz
        rospy.Subscriber("/cmd", Twist, self.velCallback)
        self.pub = rospy.Publisher('/simStates', Twist, queue_size=10)




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
        walk_mpc_idqp(vx = des_vx, vrz = des_wz)
        # print("Des vel = [ ", des_vx, ", ", des_wz, " ]" )



        

if __name__ == '__main__':

    node_instance = node()

    while not rospy.is_shutdown():

        try:
            node_instance.read_and_publish_tlm()
        except rospy.ROSInterruptException:
            pass

        node_instance.rate.sleep()