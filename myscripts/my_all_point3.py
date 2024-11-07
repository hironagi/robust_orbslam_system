#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
import std_msgs.msg
import numpy as np
import random

# def all_point_call_back(data):
#     global all_points
#     #orb_points = np.array(list(pc2.read_points(data)))
#     all_points = data
    
# def hold_camerapose_call_back(data):
#     global hold_camera_pose
#     hold_camera_pose = data

def tracked_point_call_back(data):
    global tracked_points
    tracked_points = data

    
if __name__ == "__main__":

    #initialize node
    rospy.init_node("my_all_point_cloud_pub")

    pub = rospy.Publisher("my_all_tracked_points", PointCloud2, queue_size = 10)
    
    # initialize difference from actual position
    diff = Point()

    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "world"
    fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1)
        ]
    
    rate = rospy.Rate(10)

    hold_camera_pose = PoseStamped()
    #initialize orb_points
    all_points = PointCloud2()

    tracked_points = PointCloud2()

    fixed_all_points = []
    points = []
    
    while not rospy.is_shutdown():
        #rospy.Subscriber("hold_camera_pose_pub", PoseStamped, hold_camerapose_call_back)
        #rospy.Subscriber("/orb_slam3/all_points", PointCloud2, all_point_call_back)
        # subscribe fixed tracked point
        rospy.Subscriber("my_tracked_points", PointCloud2, tracked_point_call_back)

        points = list(pc2.read_points(tracked_points, field_names=("x","y","z"), skip_nans=True))
        
        if len(points) >= 100:
            rand_selected_points = random.sample(points, 50)
        else:
            rand_selected_points = random.sample(points, len(points))
        
        fixed_all_points.extend([(x,y,z) for x,y,z in rand_selected_points])

        #fixed_all_points.extend([(x, y, z) for x, y, z in points])
        #fixed_all_points += [(x + hold_camera_pose.pose.position.x, y + hold_camera_pose.pose.position.y, z + hold_camera_pose.pose.position.z) for x, y, z in points]
        #print(len(fixed_all_points))
        #fixed_all_points = [(x + hold_camera_pose.pose.position.x, y + hold_camera_pose.pose.position.y, z + hold_camera_pose.pose.position.z) for x, y, z in points]


        my_pcl_msg = pc2.create_cloud(header, fields, list(fixed_all_points))
                
        pub.publish(my_pcl_msg)
        rate.sleep()
