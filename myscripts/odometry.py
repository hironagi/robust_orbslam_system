#!/usr/bin/env python3
import math
import rospy
import tf
from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import Bool

def tracking_state_callback(data):
    # tracking_state is a global variable that stores the tracking state
    global tracking_state
    tracking_state = data
    #print("tracking_state ----> ",tracking_state)

def camera_pose_callback(data):
    global orb_camera_pose
    orb_camera_pose = data

#def publish_camera_pose():
if __name__ == "__main__":
    # initialize node
    rospy.init_node("odom_camera_pose_publisher", anonymous=True)
    # initialize publisher
    pub = rospy.Publisher("odometry_camera_pose", PoseStamped, queue_size=10)
    # initialize subscriber
    rospy.Subscriber("/orb_slam3/tracking_state", Bool, tracking_state_callback)
    
    rospy.Subscriber("/orb_slam3/camera_pose", PoseStamped, camera_pose_callback)
    # set rospy loop rate
    rate = rospy.Rate(30)
    # initialize camera_pose
    camera_pose = PoseStamped()
    # ORBの位置は、自己位置推定が始まったかどうかのためだけに使用する。
    orb_camera_pose = PoseStamped()
    start = False
    
    # initialize tracking_state
    tracking_state = Bool()
    # set frame_id
    camera_pose.header.frame_id = "world"

    # 動画に応じた速度を設定
    #velocity = 0.167 # m/s
    velocity = 0.15 # m/s
    angular_velocity = 0.001 # rad/s

    rospy.wait_for_message("/orb_slam3/tracking_state", Bool)

    while not rospy.is_shutdown():
        # update camera pose
        # ROSのrospyライブラリを使用して、camera_poseメッセージのヘッダーのタイムスタンプを現在の時刻に設定
        camera_pose.header.stamp = rospy.Time.now()
        
        
        if(math.sqrt(orb_camera_pose.pose.position.x**2 + orb_camera_pose.pose.position.y**2 + orb_camera_pose.pose.position.z**2) != 0):
            start = True
        
        if(start):
            camera_pose.pose.position.z += velocity * rate.sleep_dur.to_sec()
            camera_pose.pose.orientation.z += angular_velocity * rate.sleep_dur.to_sec()
            print("tracking_state ----.> ",tracking_state)
            print("ループ一回の実行にかかる時間 ----> ",rate.sleep_dur.to_sec())
            print("distance ----> ",camera_pose.pose.position.z)

        # publish camera pose
        pub.publish(camera_pose)

        rate.sleep()

#if __name__ == "__main__":
#    try:
#        publish_camera_pose()
#    except rospy.ROSInterruptException:
#        pass
