#!/usr/bin/env python
import pyutils


class node(): 
    def __init__(self):
        rospy.init_node('optitrack_vis_node', anonymous=True)
        
        self.rate = rospy.Rate(5) # 10hz
        rospy.Subscriber("/vrpn_client_node/Unitree/pose", PoseStamped, self.unitreeCallback)
        rospy.Subscriber("/vrpn_client_node/Tower1/pose", PoseStamped, self.tower1Callback)
        rospy.Subscriber("/vrpn_client_node/Tower2/pose", PoseStamped, self.tower2Callback)
        rospy.Subscriber("/vrpn_client_node/Tower3/pose", PoseStamped, self.tower3Callback)
        
        self.pubUnitree = rospy.Publisher('/unitree_vis_marker',Marker, queue_size=1)
        self.pubTower1 = rospy.Publisher('/tower1_vis_marker',Marker, queue_size=1)
        self.pubTower2 = rospy.Publisher('/tower2_vis_marker',Marker, queue_size=1)
        self.pubTower3 = rospy.Publisher('/tower3_vis_marker',Marker, queue_size=1)


    def unitreeCallback(self, msg): 
        marker = self.createMarker(0,msg)
        self.pubUnitree.publish(marker)

    def tower1Callback(self, msg): 
        marker = self.createMarker(1,msg)
        self.pubTower1.publish(marker)

    def tower2Callback(self, msg): 
        marker = self.createMarker(2,msg)
        self.pubTower2.publish(marker)

    def tower3Callback(self, msg): 
        marker = self.createMarker(3,msg)
        self.pubTower3.publish(marker)



    def createMarker(self, id, pose): 
        # Add point at location pt as a marker in the marker_array
        marker = Marker()
        marker.header.frame_id = "t265_odom_frame"
        marker.type = marker.CYLINDER
        if id == 0: 
            marker.type = marker.ARROW
            
        marker.action = marker.ADD
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1
        if id == 0: 
            marker.scale.x = 0.25
            marker.scale.z = 0.1
            marker.scale.y = 0.1
        marker.color.a = 0.9
        marker.color.r = 1 
        marker.color.g = 0.25
        if id == 0: 
            marker.color.r = 0.5
            marker.color.g = 0
        marker.color.b = 0.25
        marker.lifetime = rospy.Duration(0, 10000)
        marker.pose.orientation.w = pose.pose.orientation.w
        marker.pose.orientation.z = pose.pose.orientation.z
        marker.id = id
        marker.pose.position.x = pose.pose.position.x + realsense_offset
        marker.pose.position.y = pose.pose.position.y -1
        marker.pose.position.z = 0.5
        return marker

if __name__ == '__main__':

    node_instance = node()

    rospy.spin()