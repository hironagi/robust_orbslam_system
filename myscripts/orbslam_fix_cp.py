#!/usr/bin/env python3
import tf
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
import math
from std_msgs.msg import Bool
from std_msgs.msg import Float32
import time


def orb_callback(data):
    # tracking_state is a global variable that stores the tracking state
    global orb_position
    orb_position = data
    
def odom_callback(data):
    # tracking_state is a global variable that stores the tracking state
    global odom_position
    odom_position = data
    
def tracking_state_callback(data):
    # tracking_state is a global variable that stores the tracking state
    global tracking_state
    tracking_state = data
    
if __name__ == "__main__":

    #initialize node
    rospy.init_node("fixed_orb_slam", anonymous=True)
    # パブリッシャーの作成
    pub = rospy.Publisher("camera_pose_pub", PoseStamped, queue_size=10)
    # 縮尺をパブリッシュするためのパブリッシャの作成
    pub_scale = rospy.Publisher("scale", Float32, queue_size=10)
    # カメラの位置を受け取るためのサブスクライバの作成
    rospy.Subscriber("/orb_slam3/camera_pose", PoseStamped, orb_callback)
    # オドメトリによるカメラ位置を受けとるためのサブスクライバの作成
    rospy.Subscriber("odometry_camera_pose", PoseStamped, odom_callback)
    # トラッキング状態を受け取るためのサブスクライバの作成
    #rospy.Subscriber("/orb_slam3/tracking_state", Bool, tracking_state_callback)

    # ループの周期を設定
    rate = rospy.Rate(30)
    # initialize orb_position
    orb_position = PoseStamped()
    # initialize odom_position
    odom_position = PoseStamped()
    
    # t_i − １ のORBの位置を保持するための変数
    # initialize orb_position_prev
    orb_position_prev = PoseStamped()
    
    # t_i − １ のOdom（真値）の位置を保持するための変数
    # initialize odom_position_prev
    odom_position_prev = PoseStamped()
    
    # t_i の修正したORBの位置を保持するための変数
    fixed_orb_position = PoseStamped()
    fixed_orb_position.header = orb_position.header
    fixed_orb_position.pose.orientation = orb_position.pose.orientation
    
    # initialize R
    R = 3.0
    # 3σの範囲内に収まるようにするための変数
    #upper_limit = 3.69
    #lower_limit = 2.64
    
    tracking_state = Bool()
    tracking_state_prev = Bool()
    switch = False
    
    # 開始時刻を獲得
    t1 = time.time()

    while not rospy.is_shutdown():
        t2 = time.time()
        
        fixed_orb_position.header = orb_position.header   
        fixed_orb_position.pose.orientation = orb_position.pose.orientation
        
        # odom_position と odom_position_prev から移動距離r_i'(odom_distance)を計算
        odom_distance = math.sqrt((odom_position.pose.position.x - odom_position_prev.pose.position.x)**2 + (odom_position.pose.position.y - odom_position_prev.pose.position.y)**2 + (odom_position.pose.position.z - odom_position_prev.pose.position.z)**2)
        
        # orb_position と orb_position_prev から移動距離r_i(orb_distance)を計算
        orb_distance = math.sqrt((orb_position.pose.position.x - orb_position_prev.pose.position.x)**2 + (orb_position.pose.position.y - orb_position_prev.pose.position.y)**2 + (orb_position.pose.position.z - orb_position_prev.pose.position.z)**2)
        
        # # 10秒経過したら縮尺Rを計算
        # odom_distance_tensecond = math.sqrt(odom_position.pose.position.x**2 + odom_position.pose.position.y**2 + odom_position.pose.position.z**2)
        # orb_distance_tensecond = math.sqrt(orb_position.pose.position.x**2 + orb_position.pose.position.y**2 + orb_position.pose.position.z**2)
        
        #R = 3.0
        
        if t2 - t1 > 2 and odom_distance != 0 and orb_distance != 0 and switch == True:
            R = odom_distance / orb_distance
            # fixed_orb_position.pose.position.x = orb_position.pose.position.x * R
            # fixed_orb_position.pose.position.y = orb_position.pose.position.y * R
            # fixed_orb_position.pose.position.z = orb_position.pose.position.z * R
            print("R -->", R, "  odom_distance -->", odom_distance, "  orb_distance -->", orb_distance)
            t1 = t2
            odom_position_prev = odom_position
            orb_position_prev = orb_position
            switch = False
            
        if tracking_state.data == True and tracking_state_prev == False:
            R = 7.0
            
        
        # if(odom_distance != 0 and orb_distance != 0):
        #     # r_i' と r_i から縮尺Rを計算
        #     R = odom_distance / orb_distance
        
        # orb_position
        fixed_orb_position.pose.position.x = orb_position.pose.position.x * R
        fixed_orb_position.pose.position.y = orb_position.pose.position.y * R
        fixed_orb_position.pose.position.z = orb_position.pose.position.z * R
        
        if(t2 - t1 > 1):
            print("now time:",t2)
            print(" fixed_orb_position.pose.position:\n",fixed_orb_position.pose.position)
            print("-------------------------------------------")
            t1 = t2
        # odom_position_prev と orb_position_prev を更新
        # odom_position_prev = odom_position
        # orb_position_prev = orb_position
        
        #rint("R -->", R, "  odom_distance -->", odom_distance, "  orb_distance -->", orb_distance)
        # publish fixed_orb_position
        pub.publish(fixed_orb_position)
        pub_scale.publish(Float32(R))
        #print()
        tracking_state_prev = tracking_state.data
        rate.sleep()