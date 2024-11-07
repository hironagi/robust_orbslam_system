#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def marker_call_back(data):
    global new_point
    new_point = data.pose.position

if __name__ == "__main__":

    #initialize node
    rospy.init_node("marker_pub")
    #Create a pubilsher
    pub = rospy.Publisher("my_kf_marker_pub", Marker, queue_size = 10)

    #Define marker
    marker_data = Marker()
    marker_data.header.frame_id = "world"
    marker_data.header.stamp = rospy.Time.now()
    marker_data.ns = "kf_markers"
    marker_data.id = 0
    marker_data.type = 7
    marker_data.action = Marker.ADD

    marker_data.pose.position.x = 0.0
    marker_data.pose.position.y = 0.0
    marker_data.pose.position.z = 0.0

    marker_data.pose.orientation.x=0.0
    marker_data.pose.orientation.y=0.0
    marker_data.pose.orientation.z=0.0
    marker_data.pose.orientation.w=1.0

    #scale
    marker_data.scale.x = 0.05
    marker_data.scale.y = 0.05
    marker_data.scale.z = 0.05

    #color
    marker_data.color.r = 0.0
    marker_data.color.g = 0.0
    marker_data.color.b = 1.0
    marker_data.color.a = 1.0

    #lifetime: Maker display time
    marker_data.lifetime = rospy.Duration()

    #frame_locked:
    marker_data.frame_locked = False
    
    rate = rospy.Rate(50)
    
    #initialize points
    marker_data.points = []
    
    new_point = Point()

    while not rospy.is_shutdown():
        rospy.Subscriber("camera_pose_pub", PoseStamped, marker_call_back)
        
        #add marker points
        marker_data.points.append(new_point)
        
        #print(marker_data.points)
    
        pub.publish(marker_data)
        
        rate.sleep()

