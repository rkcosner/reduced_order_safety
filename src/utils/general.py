from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

realsense_offset = 0.26 



# Create Visualization Markers
def createMarker(self, id, pose, exists): 
    marker = Marker()

    # The received positions are in the world frame, 
    # so we have to shift them forward by the realsense 
    # offset to get them in the realsense reference frame
    marker.header.frame_id = "t265_odom_frame"
    marker.pose.position.x = pose.pose.position.x + realsense_offset
    marker.pose.position.y = pose.pose.position.y -1
    marker.pose.position.z = 0.5
    
    # Unitree 
    if id == 0: 
        marker.type = marker.ARROW
        marker.scale.x = 0.25
        marker.scale.z = 0.1
        marker.scale.y = 0.1
        marker.color.r = 0.5
        marker.color.g = 0
    # Obstacle   
    else: 
        marker.type = marker.CYLINDER
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1
        marker.color.r = 1 
        marker.color.g = 0.25

    marker.color.b = 0.25
    marker.color.a = 0.9
    marker.lifetime = rospy.Duration(0, 10000)
    marker.pose.orientation.w = pose.pose.orientation.w
    marker.pose.orientation.z = pose.pose.orientation.z
    marker.id = id
    
    if not exists: 
        marker.action = marker.ADD
    else: 
        marker.action = marker.MODIFY

    return marker