#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
import numpy as np

realsense_offset = 0.26 
robot_radius = 0.32
optitrack_adjust_y = 0

# Node update rates
optitrack_vis_freq = 10
unitree_bridge_freq = 20
controller_freq = 20

###########################################################
# Problem Params
xgoal = np.array([[4,-1.5]]).T
dim = len(xgoal)
xO =  np.empty((0,0)) #
xOsim = np.array([[1.5,0],[3, -2]]).T #np.array([[ 2.18142767, -2.02558239],[ 2.83602033, -0.31468012]]).T##
meas_err = -0.1*np.array([[0,0],[1,1]])#np.random.normal(0,0.1,(2,2))
DO = 0.5 + robot_radius#np.empty((0))

# Controller Params
scale = 0.3
Kp = 0.2*scale
Kv = 0.08*scale
delta = 0.25
Kom = 0.6*scale
R = 0.25


visual_offset = 0.26 #m 



# Create Visualization Markers
def createMarker(id, pose, exists): 
    marker = Marker()

    # The received positions are in the world frame, 
    # so we have to shift them forward by the realsense 
    # offset to get them in the realsense reference frame
    marker.header.frame_id = "t265_odom_frame"
    marker.pose.position.x = pose[0]
    marker.pose.position.y = pose[1]
    marker.pose.position.z = 0
    
    # Unitree Optitrack
    if id == 0: 
        marker.type = marker.ARROW
        marker.scale.x = 0.25
        marker.scale.z = 0.1
        marker.scale.y = 0.1
        marker.color.r = 0.5
        marker.color.g = 0
    # Obstacle Optitrack
    elif id < 4: 
        marker.type = marker.CYLINDER
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1
        marker.color.r = 1 
        marker.color.g = 0.25
    # Unitree Onboard
    else: 
        marker.type = marker.ARROW
        marker.scale.x = 0.25
        marker.scale.z = 0.1
        marker.scale.y = 0.1
        marker.color.r = 0.5
        marker.color.g = 1

    marker.color.b = 0.25
    marker.color.a = 0.9
    marker.lifetime = rospy.Duration(0, 10000)
    marker.pose.orientation.w = np.cos(pose[2]/2)
    marker.pose.orientation.z = np.sin(pose[2]/2)
    marker.id = id
    
    if not exists: 
        marker.action = marker.ADD
    else: 
        marker.action = marker.MODIFY

    return marker

def getTimeNow(): 
    now = rospy.get_rostime()
    sec = now.secs
    nsec = now.nsecs*10**(-9) 
    return sec + nsec