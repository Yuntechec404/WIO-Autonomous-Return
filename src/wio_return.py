#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
import tf.transformations as tf
import math
import numpy as np

class PoseRecorderAndReturn:
    def __init__(self):
        self.is_recording = True
        # 儲存位姿的陣列
        self.poses = []
        # 當前位姿（用於閉環控制回饋）
        self.current_pose = None
        self.last_recorded_pose = None
        self.last_x_target = 0.0
        self.last_y_target = 0.0
        
        
        # 初始化 ROS 節點
        rospy.init_node('pose_recorder_and_return', anonymous=True)
        self.r = int(rospy.get_param("~rate", 10))
        self.rate = rospy.Rate(self.r)
        # PID 參數
        self.kp_linear = rospy.get_param("~kp_linear", 1.0)     # 5.0
        self.kp_angular = rospy.get_param("~kp_angular", 1.0)   # 0.76
        
        
        # 訂閱 Odometry 用於記錄 poses
        rospy.Subscriber('/odometry/differential_mode', Odometry, self.odometry_callback)
        
        # 訂閱 Odometry 用於更新當前位姿
        rospy.Subscriber('/odometry/differential_mode', Odometry, self.update_current_pose)
        
        # 發佈者
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # 服務
        rospy.Service('return_start', Trigger, self.handle_service_request)
        
        rospy.loginfo("Node initialized. Recording poses and waiting for service call...")
    
    def odometry_callback(self, msg):
        if self.is_recording:
            # 從 pose.pose 提取 (x, y, yaw)
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
            yaw = tf.euler_from_quaternion(quaternion)[2]
            
            # Initialization
            if self.last_recorded_pose is None:
                self.last_recorded_pose = (x, y, quaternion)
                return

            current_pose = (x, y, quaternion)
            last_x, last_y, last_q = self.last_recorded_pose
            last_yaw = tf.euler_from_quaternion(last_q)[2]
            distance = math.sqrt((x-last_x)**2 + (y-last_y)**2)
            angle_diff = abs(yaw - last_yaw)

            if distance > 0.1 or angle_diff > 0.01:
                self.poses.append(current_pose)
                self.last_recorded_pose = current_pose
                rospy.loginfo(f"Recorded pose: {current_pose}")
    
    def update_current_pose(self, msg):
        # 更新當前位姿用於閉環控制
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        yaw = tf.euler_from_quaternion(quaternion)[2]
        self.current_pose = [x, y, quaternion]
    
    def adjust_yaw(self, target_q):
        cmd = Twist()
        tolerance = 0.02  # 角度容忍度（弧度，約 1.15 度）
        max_iterations = 1000  # 最大迭代次數，防止無限迴圈
        iteration = 0

        while not rospy.is_shutdown() and iteration < max_iterations:
            # 檢查當前姿態是否可用
            if self.current_pose is None:
                rospy.logwarn("當前姿態未初始化")
                continue

            # 步驟 1: 獲取當前和目標的 yaw 角度
            current_yaw = tf.euler_from_quaternion(self.current_pose[2])[2]
            target_yaw = tf.euler_from_quaternion(target_q)[2]

            # 步驟 2: 計算 yaw 誤差
            error = target_yaw - current_yaw

            # 將誤差限制在 [-π, π] 範圍內
            error = (error + np.pi) % (2 * np.pi) - np.pi

            # 步驟 3: 檢查是否達到目標
            if abs(error) <= tolerance:
                break

            # 步驟 4: 根據誤差計算角速度
            cmd.angular.z = self.kp_angular * error
            self.pub.publish(cmd)
            self.rate.sleep()
            iteration += 1

        # 停止旋轉
        cmd.angular.z = 0.0
        self.pub.publish(cmd)
        rospy.loginfo("已達到目標 yaw 角度")
    
    def move_to_point(self, target_pose):
        x_target, y_target, q_target = target_pose
        
        # 第一步：調整朝向
        self.adjust_yaw(q_target)
        
        # 第二步：直線移動
        cmd = Twist()
        tolerance = 0.05  # 距離容忍度
        
        distance = math.sqrt((x_target - self.last_x_target)**2 + 
                            (y_target - self.last_y_target)**2)
        if distance > tolerance:
              
        # target_angle = math.atan2(y_target - self.current_pose[1], 
        #                         x_target - self.current_pose[0])
        # angular_error = target_angle - self.current_pose[2]
        
        # cmd.linear.x = -self.kp_linear * distance
        # cmd.angular.z = self.kp_angular * angular_error
            count = 7
            while count > 0:
                count = count - 1
                cmd.linear.x = -self.kp_linear * 0.3
                cmd.angular.z = 0.0
                self.pub.publish(cmd)
                self.rate.sleep()
        self.last_x_target = x_target
        self.last_y_target = y_target
    
    def return_to_start(self):
        self.is_recording = False
        reversed_poses = self.poses[::-1]
        rospy.loginfo("Starting return to origin...")
        
        for i in range(len(reversed_poses) - 1):
            target_pose = reversed_poses[i + 1]
            self.move_to_point(target_pose, self.rate)
            rospy.loginfo(f"Moved to: {target_pose}")
        
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.pub.publish(cmd)
        rospy.loginfo("Returned to start!")
    
    def handle_service_request(self, req):
        rospy.loginfo("Service called!")
        self.return_to_start()
        return TriggerResponse(success=True, message="Service executed successfully.")
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = PoseRecorderAndReturn()
    node.run()