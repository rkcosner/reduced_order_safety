#!/usr/bin/env python
import rospkg
import sys 
rospack = rospkg.RosPack()
sys.path.append(rospack.get_path('red_ord_unitree')+'/src/utils/')
from optitrack_utils import * 


if __name__ == '__main__':

    # Create Node
    optitrack_vis_node()

    # Run callbacks indefinitely
    rospy.spin()