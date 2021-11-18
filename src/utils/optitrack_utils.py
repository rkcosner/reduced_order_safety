#!/usr/bin/env python
import rospy
import rospkg
import sys 
rospack = rospkg.RosPack()
sys.path.append(rospack.get_path('red_ord_unitree')+'/src/utils/')
from general import *

class optitrack_vis_node(): 
    def __init__(self):

        # Initialize Node 
        rospy.init_node('optitrack_vis_node', anonymous=True)
        
        # Set up Publishers and subscribers
        self.rate = rospy.Rate(10) 
        rospy.Subscriber("/vrpn_client_node/Unitree/pose", PoseStamped, self.unitreeCallback)
        rospy.Subscriber("/vrpn_client_node/Tower1/pose", PoseStamped, self.tower1Callback)
        rospy.Subscriber("/vrpn_client_node/Tower2/pose", PoseStamped, self.tower2Callback)
        rospy.Subscriber("/vrpn_client_node/Tower3/pose", PoseStamped, self.tower3Callback)
        
        self.pubUnitree = rospy.Publisher('/unitree_vis_marker',Marker, queue_size=1)
        self.pubTower1 = rospy.Publisher('/tower1_vis_marker',Marker, queue_size=1)
        self.pubTower2 = rospy.Publisher('/tower2_vis_marker',Marker, queue_size=1)
        self.pubTower3 = rospy.Publisher('/tower3_vis_marker',Marker, queue_size=1)

        # Modify or Create Marker
        self.unitreeExists = False
        self.tower1Exists = False
        self.tower2Exists = False
        self.tower3Exists = False

    # Callbacks to update visualization markers
    def unitreeCallback(self, msg): 
        marker = createMarker(0,msg,self.unitreeExists)
        self.pubUnitree.publish(marker)
        if not self.unitreeExists:
            self.unitreeExists = True

    def tower1Callback(self, msg): 
        marker = createMarker(1,msg, self.tower1Exists) 
        self.pubTower1.publish(marker)
        if not self.tower1Exists:
            self.tower1Exists = True

    def tower2Callback(self, msg): 
        marker = createMarker(2,msg, self.tower2Exists)
        self.pubTower2.publish(marker)
        if not self.tower2Exists:
            self.towe2Exists = True

    def tower3Callback(self, msg): 
        marker = createMarker(3,msg, self.tower3Exists)
        self.pubTower3.publish(marker)
        if not self.tower3Exists:
            self.tower3Exists = True
