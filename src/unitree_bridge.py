#!/usr/bin/env python
import rospkg
import sys 
rospack = rospkg.RosPack()
sys.path.append(rospack.get_path('red_ord_unitree')+'/src/utils/')

from unitree_bridge_utils import * 
from subprocess import Popen


if __name__ == '__main__':

    experiment_type = rospy.get_param("/safe_velocity_node/experiment_type", 0)

    isSim = (experiment_type == 0)

    print("Is Sim : ", isSim)

    bridge_node = unitree_bridge_node(isSim)
    while not rospy.is_shutdown():
        try:
            bridge_node.read_and_publish_tlm()
        except rospy.ROSInterruptException:
            pass
        
        bridge_node.rate.sleep()
