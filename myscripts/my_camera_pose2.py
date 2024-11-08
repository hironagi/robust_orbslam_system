#!/usr/bin/env python3
import tf
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
import math
from std_msgs.msg import Bool
import numpy as np
from pyquaternion import Quaternion
from scipy.spatial.transform import Rotation as R
import time

def quaternion_to_euler(q):
    # クォータニオンを回転行列に変換
    rotation = R.from_quat([q[0], q[1], q[2], q[3]])
    # 回転行列をオイラー角に変換
    euler = rotation.as_euler('xyz', degrees=True)
    return euler

def tracking_state_callback(data):
    # tracking_state is a global variable that stores the tracking state
    global tracking_state
    tracking_state = data

def pose_callback(data):
    # global read_camera_pose
    global hold_camera_pose
    global camera_pose
    global count
    global tmp
    global tracking_state_prev
    global read_camera_pose

    read_camera_pose = data

def odometry_callback(data):
    global odometry_camera_pose
    odometry_camera_pose = data

def camera_odom_filter(odom_pose, read_cammera_pose, hold_camera_pose):
    w1 = 0.9
    w2 = 0.1
    filtered_pose = PoseStamped()
    filtered_pose.pose.position.x = w1 * odom_pose.pose.position.x + w2 * (read_cammera_pose.pose.position.x + hold_camera_pose.pose.position.x)
    filtered_pose.pose.position.y = w1 * odom_pose.pose.position.y + w2 * (read_cammera_pose.pose.position.y + hold_camera_pose.pose.position.y)
    filtered_pose.pose.position.z = w1 * odom_pose.pose.position.z + w2 * (read_cammera_pose.pose.position.z + hold_camera_pose.pose.position.z)
    
    filtered_pose.pose.orientation.x = w1 * odom_pose.pose.orientation.x + w2 * read_cammera_pose.pose.orientation.x
    filtered_pose.pose.orientation.y = w1 * odom_pose.pose.orientation.y + w2 * read_cammera_pose.pose.orientation.y
    filtered_pose.pose.orientation.z = w1 * odom_pose.pose.orientation.z + w2 * read_cammera_pose.pose.orientation.z
    filtered_pose.pose.orientation.w = w1 * odom_pose.pose.orientation.w + w2 * read_cammera_pose.pose.orientation.w
    return filtered_pose

if __name__ == "__main__":
    count = 0
    #initialize node
    rospy.init_node("pose_calculater", anonymous=True)
    # パブリッシャーの作成
    pub = rospy.Publisher("camera_pose_pub", PoseStamped, queue_size=10)
    pub2 = rospy.Publisher("hold_camera_pose_pub", PoseStamped, queue_size=10)
    # カメラの位置を受け取るためのサブスクライバの作成
    rospy.Subscriber("fixed_orb_position", PoseStamped, pose_callback)
    # トラッキング状態を受け取るためのサブスクライバの作成
    rospy.Subscriber("/orb_slam3/tracking_state", Bool, tracking_state_callback)
    # オドメトリによるカメラ位置を受けとるためのサブスクライバの作成
    rospy.Subscriber("odometry_camera_pose", PoseStamped, odometry_callback)

    # ループの周期を設定
    rate = rospy.Rate(30)
    # initialize read_camera_pose
    read_camera_pose = PoseStamped()
    # initialize camera_pose
    camera_pose = PoseStamped()
    # initialize hold_camera_pose
    hold_camera_pose = PoseStamped()
    # initialize tracking_state
    tracking_state = Bool()
    # initialize tracking_state_prev
    tracking_state_prev = Bool()
    # initialize tmp
    tmp = PoseStamped()
    # initialize odometry_camera_pose
    odometry_camera_pose = PoseStamped()
    begin_odometry_camera_pose = PoseStamped()
    camera_odom_orientation = Quaternion()
    diff_odom_pose_x = 0
    diff_odom_pose_y = 0
    diff_odom_pose_z = 0

    camera_pose = read_camera_pose

    rospy.wait_for_message("/orb_slam3/camera_pose", PoseStamped)
    rospy.wait_for_message("/orb_slam3/tracking_state", Bool)
    
    t1 = time.time()

    while not rospy.is_shutdown():
        t2 = time.time()

        #rospy.Subscriber("/orb_slam3/camera_pose", PoseStamped, pose_callback)
        #rospy.Subscriber("/orb_slam3/tracking_state", Bool, tracking_state_callback)

        #print(read_camera_pose.pose.position.z)
        #store in x,y,z coordinates respectively from the received position data
        #pos_z = read_camera_pose.pose.position.z

        # 自己位置を喪失するまで、その時点でのオドメトリのカメラの位置を保持しておく
        # if tracking_state.data == True and tracking_state_prev == True and read_camera_pose.pose.position.z != 0:
        #     begin_odometry_camera_pose = odometry_camera_pose

        if tracking_state.data == False and tracking_state_prev == True:
           begin_odometry_camera_pose = odometry_camera_pose
        # 自己位置を再び取得したら、その時点でのオドメトリのカメラの位置との差分を計算する。
        elif tracking_state.data == True and tracking_state_prev == False:  
            diff_odom_pose_x = odometry_camera_pose.pose.position.x - begin_odometry_camera_pose.pose.position.x
            diff_odom_pose_y = odometry_camera_pose.pose.position.y - begin_odometry_camera_pose.pose.position.y
            diff_odom_pose_z = odometry_camera_pose.pose.position.z - begin_odometry_camera_pose.pose.position.z

            # カメラの向きは自己位置を喪失する前のものを保持する
            camera_odom_orientation = begin_odometry_camera_pose.pose.orientation
            #camera_odom_orientation = odometry_camera_pose.pose.orientation
            # print("diff_odom_pose_x -->",diff_odom_pose_x, "  diff_odom_pose_y -->", diff_odom_pose_y, "  diff_odom_pose_z -->", diff_odom_pose_z)
            #print("camera_odom_orientation -->",camera_odom_orientation)
        
        # 途切れるまでカメラの位置を更新し続ける
        #print("tmppos_z -->",tmp.pose.position.z, "  tracking_state -->", tracking_state, "  tracking_state_prev -->", tracking_state_prev)

        # もし tracking_state.data が true かつ tracking_state_prev が true ならば、カメラはトラッキングを継続しており、カメラの位置を更新する
        if tracking_state.data == True and tracking_state_prev == True and read_camera_pose.pose.position.z != 0:
            #フィルターをかけずORBによるカメラの位置をtmpに入れる場合、以下のようになる
            #もしodometry_callbackが呼び出されていないなら、
            # if(odometry_callback == None):
            #     tmp = read_camera_pose
            #     camera_pose.header = read_camera_pose.header   
            #     camera_pose.pose.orientation = read_camera_pose.pose.orientation
            #     #フィルターをかけずにORBによるカメラの位置を更新する場合、以下のようになる
            #     camera_pose.pose.position.x = hold_camera_pose.pose.position.x + read_camera_pose.pose.position.x 
            #     camera_pose.pose.position.y = hold_camera_pose.pose.position.y + read_camera_pose.pose.position.y
            #     camera_pose.pose.position.z = hold_camera_pose.pose.position.z + read_camera_pose.pose.position.z 
            # else:
            # #オドメトリによる継続的なフィルターをかけたカメラの位置をtmpに入れる場合、以下のようになる
            #     tmp = camera_odom_filter(odometry_camera_pose, read_camera_pose, hold_camera_pose)
            #     camera_pose.header = read_camera_pose.header     
            #     camera_pose.pose.orientation = camera_odom_filter(odometry_camera_pose, read_camera_pose, hold_camera_pose).pose.orientation
            #     #オドメトリによる継続的なフィルターをかけたカメラの位置を更新する場合、以下のようになる
            #     camera_pose.pose.position.x = camera_odom_filter(odometry_camera_pose, read_camera_pose, hold_camera_pose).pose.position.x
            #     camera_pose.pose.position.y = camera_odom_filter(odometry_camera_pose, read_camera_pose, hold_camera_pose).pose.position.y
            #     camera_pose.pose.position.z = camera_odom_filter(odometry_camera_pose, read_camera_pose, hold_camera_pose).pose.position.z
                
            
            tmp = read_camera_pose
            camera_pose.header = read_camera_pose.header   
            camera_pose.pose.orientation = read_camera_pose.pose.orientation
            #フィルターをかけずにORBによるカメラの位置を更新する場合、以下のようになる
            camera_pose.pose.position.x = hold_camera_pose.pose.position.x + read_camera_pose.pose.position.x 
            camera_pose.pose.position.y = hold_camera_pose.pose.position.y + read_camera_pose.pose.position.y
            camera_pose.pose.position.z = hold_camera_pose.pose.position.z + read_camera_pose.pose.position.z         

            #print("camera_pose.z - odometry.z = ",camera_pose.pose.position.z - odometry_camera_pose.pose.position.z)
            # print("camera_pose.z = ",camera_pose.pose.position.z)

        #if the coordinates are reset 
        # カメラが再度トラッキングを開始したとき、チェックポイントを更新する
        # tracking_state.data が true かつ tracking_state_prev が false のとき、カメラが再度トラッキングを開始したと判断し、カメラの位置を保持する
        if tracking_state.data == True and tracking_state_prev == False:
            print("relocated!\n")
            camera_pose.header = read_camera_pose.header 
            # 復帰後のカメラの向きは信用できないので、復帰時点でのオドメトリのカメラの向きを保持する
            #camera_pose.pose.orientation = read_camera_pose.pose.orientation
            #camera_pose.pose.orientation = odometry_camera_pose.pose.orientation

            hold_camera_pose.pose.position.x += tmp.pose.position.x + diff_odom_pose_x
            hold_camera_pose.pose.position.y += tmp.pose.position.y + diff_odom_pose_y
            hold_camera_pose.pose.position.z += tmp.pose.position.z + diff_odom_pose_z
            # print("tmp_pos_z",tmp.pose.position.z)
            # print("diff_odom_pose_z",diff_odom_pose_z)
            # print("hold_camera_pos_z",hold_camera_pose.pose.position.z)
            # print("----------------------------------------------------")
            #print("tmp_pos_z",tmp.pose.position.z)
            #print("hold_camera_pos_z",hold_camera_pose.pose.position.z)
            
    
            #camera_pose.header = read_camera_pose.header
            #camera_pose.pose.orientation = read_camera_pose.pose.orientation
            
            #****フィルターをかけずにORBによるカメラの位置を更新する場合、以下のようになる****
            # もしodometry_callbackが呼び出されていないなら、
            # if(odometry_callback == None):
            #     camera_pose.pose.position.x = hold_camera_pose.pose.position.x + read_camera_pose.pose.position.x
            #     camera_pose.pose.position.y = hold_camera_pose.pose.position.y + read_camera_pose.pose.position.y
            #     camera_pose.pose.position.z = hold_camera_pose.pose.position.z + read_camera_pose.pose.position.z 
            # # もしodometry_callbackが呼び出されているなら、
            # else:
            # #****オドメトリによる継続的なフィルターをかけたカメラの位置を更新する場合、以下のようになる****
            #     camera_pose.pose.position.x = camera_odom_filter(odometry_camera_pose, read_camera_pose, hold_camera_pose).pose.position.x
            #     camera_pose.pose.position.y = camera_odom_filter(odometry_camera_pose, read_camera_pose, hold_camera_pose).pose.position.y
            #     camera_pose.pose.position.z = camera_odom_filter(odometry_camera_pose, read_camera_pose, hold_camera_pose).pose.position.z
            # print("camera_odom_filter",camera_odom_filter(odometry_camera_pose, read_camera_pose, hold_camera_pose).pose.position.z)
            # print("camera_pose_z",camera_pose.pose.position.z)
            # print("----------------------------------------------------")
            
            camera_pose.pose.position.x = hold_camera_pose.pose.position.x + read_camera_pose.pose.position.x
            camera_pose.pose.position.y = hold_camera_pose.pose.position.y + read_camera_pose.pose.position.y
            camera_pose.pose.position.z = hold_camera_pose.pose.position.z + read_camera_pose.pose.position.z 
            
    
        tracking_state_prev = tracking_state.data

        x = camera_pose.pose.orientation.x
        y = camera_pose.pose.orientation.y
        z = camera_pose.pose.orientation.z
        w = camera_pose.pose.orientation.w
        
        if(t2 - t1 > 1):
            print("now time:",t2)
            print(" fixed_orb_position.pose.position:\n",camera_pose.pose.position)
            print("-------------------------------------------")
            t1 = t2
            
        #もしカメラの向きのノルムが0出ないなら以下の処理を行う
        # if math.sqrt(x**2 + y**2 + z**2 + w**2) != 0:
        #     print(quaternion_to_euler([x,y,z,w]))
        pub.publish(camera_pose)
        pub2.publish(hold_camera_pose)
        
        rate.sleep()
