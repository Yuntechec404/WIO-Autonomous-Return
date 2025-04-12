#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
import tf.transformations as tf
import math

class DifferentialOdom:
    def __init__(self):
        # 初始化 ROS 節點
        rospy.init_node('differential_mode_node', anonymous=True)
        self.rate = rospy.Rate(10)  # 10 Hz
        
        # 當前位置和偏航角
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        # 前一時刻的位置和偏航角
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_yaw = 0.0
        
        # 標記是否為第一筆數據
        self.is_first_msg = True
        
        # 訂閱 odometry/filtered 主題
        rospy.Subscriber('odometry/filtered', Odometry, self.callback)
        
        # 發佈獨立計算的里程計資料
        self.pub = rospy.Publisher('odometry/differential_mode', Odometry, queue_size=10)

    def callback(self, odometry_msg):
        # 從消息中提取當前位置和偏航角
        current_x = odometry_msg.pose.pose.position.x
        current_y = odometry_msg.pose.pose.position.y
        current_yaw = self.get_yaw(odometry_msg.pose.pose.orientation)
        
        # 如果是第一筆數據，初始化並返回
        if self.is_first_msg:
            self.x = current_x
            self.y = current_y
            self.yaw = current_yaw
            self.last_x = current_x
            self.last_y = current_y
            self.last_yaw = current_yaw
            self.is_first_msg = False
            return

        # 計算位置差
        delta_x = current_x - self.last_x
        delta_y = current_y - self.last_y
        
        # 計算位置差在偏航角方向上的投影
        projection = delta_x * math.cos(self.last_yaw) + delta_y * math.sin(self.last_yaw)
        
        # 判斷移動方向
        direction = 1 if projection >= 0 else -1  # 向前: 1，向後: -1
        
        # 計算位移幅度
        displacement_magnitude = math.sqrt(delta_x**2 + delta_y**2)
        
        # 根據方向調整位移
        displacement = direction * displacement_magnitude
        
        # 計算全局坐標系中的 dx 和 dy
        dx = displacement * math.cos(self.last_yaw)
        dy = displacement * math.sin(self.last_yaw)
        
        # 靜止檢測：若位移過小，設為 0
        threshold = 0.01
        if abs(dx) < threshold and abs(dy) < threshold:
            dx = 0.0
            dy = 0.0
        
        # 累積位移到總位置
        self.x += dx
        self.y += dy
        self.yaw = current_yaw
        
        # 更新前一時刻的數據
        self.last_x = current_x
        self.last_y = current_y
        self.last_yaw = current_yaw
        
        # 記錄日誌
        # rospy.loginfo("Differential:\nX: %.3f m\nY: %.3f m\nYaw: %.2f deg", 
        #               self.x, self.y, self.yaw * 180.0 / math.pi)
        
        # 構造並發佈新的里程計消息
        new_odom_msg = Odometry()
        new_odom_msg.header = odometry_msg.header
        new_odom_msg.pose.pose.position.x = self.x
        new_odom_msg.pose.pose.position.y = self.y
        new_odom_msg.pose.pose.position.z = odometry_msg.pose.pose.position.z
        new_odom_msg.pose.pose.orientation = odometry_msg.pose.pose.orientation
        self.pub.publish(new_odom_msg)

    def get_yaw(self, orientation):
        # 從四元數轉換為偏航角
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = tf.euler_from_quaternion(quaternion)
        return yaw
    
    def run(self):
        # 主循環
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = DifferentialOdom()
        node.run()
    except rospy.ROSInterruptException as e:
        print(e)