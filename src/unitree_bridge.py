#!/usr/bin/env python
import rospkg
import sys 
rospack = rospkg.RosPack()
sys.path.append(rospack.get_path('red_ord_unitree')+'/src/utils/')

from unitree_bridge_utils import * 



if __name__ == '__main__':

    node_instance = unitree_bridge_node()

    while not rospy.is_shutdown():

        try:
            node_instance.read_and_publish_tlm()
        except rospy.ROSInterruptException:
            pass

        node_instance.rate.sleep()
