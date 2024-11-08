#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
import std_msgs.msg
from std_msgs.msg import Float32
import numpy as np
from scipy.spatial.transform import Rotation as R
import math

def point_call_back(data):
    global orb_points
    #orb_points = np.array(list(pc2.read_points(data)))
    orb_points = data

def my_camerapose_call_back(data):
    global camera_pose
    camera_pose = data

def orb_camerapose_call_back(data):
    global orb_camera_pose
    orb_camrera_pose = data

def scale_call_back(data):
    global scale
    scale = data

def quaternion_to_euler(q):
    return R.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz', degrees=True)

def Rotation_xyz(pointcloud, theta_x, theta_y, theta_z):
    theta_x = math.radians(theta_x)
    theta_y = math.radians(theta_y)
    theta_z = math.radians(theta_z)
    rot_x = np.array([[ 1,                 0,                  0],
                      [ 0, math.cos(theta_x), -math.sin(theta_x)],
                      [ 0, math.sin(theta_x),  math.cos(theta_x)]])

    rot_y = np.array([[ math.cos(theta_y), 0,  math.sin(theta_y)],
                      [                 0, 1,                  0],
                      [-math.sin(theta_y), 0, math.cos(theta_y)]])

    rot_z = np.array([[ math.cos(theta_z), -math.sin(theta_z), 0],
                      [ math.sin(theta_z),  math.cos(theta_z), 0],
                      [                 0,                  0, 1]])

    rot_matrix = rot_z.dot(rot_y.dot(rot_x))
    rot_pointcloud = rot_matrix.dot(pointcloud.T).T
    return rot_pointcloud, rot_matrix



if __name__ == "__main__":

    #initialize node
    rospy.init_node("my_point_cloud_pub")
    # パブリッシャーの作成
    pub = rospy.Publisher("my_tracked_points", PointCloud2, queue_size = 10)
    # サブスクライバの作成
    # 修正されたORBの位置、修正前のORBトラックポイント、修正前のORBの位置を受け取る
    rospy.Subscriber("camera_pose_pub", PoseStamped, my_camerapose_call_back)
    rospy.Subscriber("/orb_slam3/tracked_points", PointCloud2, point_call_back)
    rospy.Subscriber("/orb_slam3/camera_pose", PoseStamped, orb_camerapose_call_back)
    rospy.Subscriber("scale", Float32, scale_call_back)

    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "world"
    fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
        PointField("rgb", 12, PointField.UINT32, 1),
        ]
    
    rate = rospy.Rate(30)

    camera_pose = PoseStamped()
    orb_camera_pose = PoseStamped()
    orb_points = PointCloud2()

    mv_points = []
    
    S = 3.0
    scale = Float32()
    
    while not rospy.is_shutdown():
        # ORBからの特徴点を取得し、リストに格納する
        points = list(pc2.read_points(orb_points, skip_nans=True))
        
        mv_points = [((x - orb_camera_pose.pose.position.x)*S + camera_pose.pose.position.x,
                     (y - orb_camera_pose.pose.position.y)*S + camera_pose.pose.position.y,
                     (z - orb_camera_pose.pose.position.z)*S + camera_pose.pose.position.z, 0xff0000) for x, y, z in points]
        S = scale.data
        
        print(len(mv_points))
        my_pcl_msg = pc2.create_cloud(header, fields, list(mv_points))
        
        mv_points = []
        pub.publish(my_pcl_msg)
        rate.sleep()

        
        
