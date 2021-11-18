#!/usr/bin/env python
import rospkg
import sys 
rospack = rospkg.RosPack()
sys.path.append(rospack.get_path('red_ord_unitree')+'/src/utils/')

from unitree_bridge_utils import * 



if __name__ == '__main__':
    
    bridge_node = unitree_bridge_node()
    while not rospy.is_shutdown():
        try:
            bridge_node.read_and_publish_tlm()
        except rospy.ROSInterruptException:
            pass
        
        bridge_node.rate.sleep()
