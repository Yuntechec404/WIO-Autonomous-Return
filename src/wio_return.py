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
        self.motion_mode = "Stop"
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
        rospy.logwarn(f"Rate: {self.r}")
        rospy.logwarn(f"Linear: {self.kp_linear}\tAngular: {self.kp_angular}")
        
        
        # 訂閱 Odometry 用於記錄 poses, 訂閱 Twist 用於紀錄運動模式
        rospy.Subscriber('/odometry/differential_mode', Odometry, self.odometry_callback)
        rospy.Subscriber('/cmd_vel', Twist, self.chatter_callback)
        
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
                self.poses.append((current_pose, self.motion_mode)) # 多增加紀錄運動模式
                self.last_recorded_pose = current_pose
                rospy.loginfo(f"Recorded pose: {current_pose, self.motion_mode}")

    def chatter_callback(self, msg):
        if msg.linear.x != 0: # 斜前斜後當成直線行駛
            self.motion_mode = "Straight"
        elif msg.linear.x == 0 and msg.angular.z != 0:
            self.motion_mode = "Rotated"
        else:
            self.motion_mode = "Stop"


    def update_current_pose(self, msg):
        # 更新當前位姿用於閉環控制
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        yaw = tf.euler_from_quaternion(quaternion)[2]
        self.current_pose = [x, y, quaternion]
    
    def calculate_yaw_difference(self, initial_quaternion, final_quaternion):
        """
        計算兩個四元數之間的相對旋轉，返回目標四元數和 yaw 角度差（用於日誌）。
        
        Args:
            initial_quaternion (list): 初始四元數 [x, y, z, w]
            final_quaternion (list): 最終四元數 [x, y, z, w]
        
        Returns:
            tuple: (目標四元數 [x, y, z, w], yaw 角度差（弧度，用於日誌）)
        """
        # 將列表轉換為 numpy 陣列以進行四元數運算
        q1 = np.array(initial_quaternion)
        q2 = np.array(final_quaternion)
        
        # 計算 q1 的逆（對於單位四元數，逆等於共軛）
        q1_inv = tf.quaternion_conjugate(q1)
        
        # 計算相對旋轉 q_diff = q2 * q1_inv
        q_diff = tf.quaternion_multiply(q2, q1_inv)
        
        # 計算目標四元數 q_target = q_diff * q1（這裡實際上 q_target = q2，因為 q_diff * q1 = q2 * q1_inv * q1 = q2）
        # 為了保持一致性，我們直接使用 q2 作為目標四元數
        q_target = q2.tolist()
        
        # 為了日誌記錄，計算 yaw 角度差（可選）
        yaw_diff = tf.euler_from_quaternion(q_diff)[2]
        # 規範化 yaw 角度差到 [-π, π]
        yaw_diff = (yaw_diff + np.pi) % (2 * np.pi) - np.pi
    
    return q_target, yaw_diff

    def adjust_yaw(self, target_q):
        cmd = Twist()
        # 角度容忍度（弧度，約 0.01745 -> 1 度） 0.02 -> 1.15 deg
        tolerance = 0.01
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
    
    def move_to_point(self, target_pose, target_mode):
        x_target, y_target, q_target = target_pose
          
        cmd = Twist()
        # tolerance = 0.05  # 距離容忍度
        # distance = math.sqrt((x_target - self.last_x_target)**2 + 
        #                     (y_target - self.last_y_target)**2)
        
        if target_mode == "Straight":
            # 直線移動邏輯  # PID 控制
            count = 7
            while count > 0:
                count -= 1
                cmd.linear.x = -self.kp_linear * 0.3
                cmd.angular.z = 0.0
                self.pub.publish(cmd)
                self.rate.sleep()
        
        if target_mode == "Rotated":
            # 旋轉移動邏輯
            self.adjust_yaw(q_target)

        self.last_x_target = x_target
        self.last_y_target = y_target
    
    def return_to_start(self):
        self.is_recording = False
        reversed_poses = self.poses[::-1]
        rospy.loginfo(f"Starting return to origin...\nRecord points: {len(reversed_poses)}")
        
        # for i in range(len(reversed_poses) - 1):
        #     target_pose, target_mode = reversed_poses[i + 1]
        #     self.move_to_point(target_pose, target_mode)
        #     rospy.loginfo(f"Moved to: {target_pose} with mode: {target_mode}")

        # 250516 計算完整旋轉角
        i = 0
        while i < len(reversed_poses):
            target_pose, target_mode = reversed_poses[i]
            
            if target_mode == "Rotated":
                # 記錄最初的旋轉位姿
                initial_pose = target_pose
                initial_quaternion = initial_pose[2]
                # 繼續檢查後續是否仍是旋轉命令
                j = i + 1
                while j < len(reversed_poses) and reversed_poses[j][1] == "Rotated":
                    j += 1
                # 獲取最終旋轉位姿（如果存在）
                if j < len(reversed_poses):
                    final_pose = reversed_poses[j][0]
                else:
                    final_pose = reversed_poses[-1][0]  # 如果全是旋轉命令，取最後一個
                final_quaternion = final_pose[2]
                
                # 計算相對旋轉四元數和 yaw 角度差（用於日誌）
                target_quaternion, yaw_diff = self.calculate_yaw_difference(initial_quaternion, final_quaternion)
                
                # 構造新的目標位姿，保留初始的 x, y，僅更新四元數
                integrated_pose = (initial_pose[0], initial_pose[1], target_quaternion)
                
                # 將整合後的旋轉位姿傳遞給 move_to_point
                self.move_to_point(integrated_pose, "Rotated")
                rospy.loginfo(f"Integrated rotation from {initial_pose} to {final_pose}, yaw_diff: {yaw_diff}")
                
                # 更新索引到下一個非旋轉命令
                i = j
            else:
                # 非旋轉命令，直接移動到目標點
                self.move_to_point(target_pose, target_mode)
                rospy.loginfo(f"Moved to: {target_pose} with mode: {target_mode}")
                i += 1
        
        # 停止移動
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.pub.publish(cmd)
        rospy.loginfo("Returned to start!")
    
    def handle_service_request(self, req):
        rospy.logwarn("User's control disconnected!")
        self.return_to_start()
        return TriggerResponse(success=True, message="Service executed successfully.")
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = PoseRecorderAndReturn()
    node.run()