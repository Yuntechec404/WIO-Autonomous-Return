#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class OdomReader:
    def __init__(self):
        # 初始化ROS節點
        rospy.init_node('odom_reader', anonymous=True)
        # 訂閱/odometry/filtered話題
        rospy.Subscriber("/odometry/differential_mode", Odometry, self.odom_callback)

    def odom_callback(self, msg):
        # 從Odometry消息中提取X和Y位移量
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # 提取姿態的四元數
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        # 將四元數轉換為歐拉角，獲取yaw角度
        (_, _, yaw) = euler_from_quaternion(orientation_list)

        # 輸出X, Y和yaw到日誌
        rospy.loginfo("Differential:\nX: %.3f m\nY: %.3f m\nYaw: %.2f deg", x, y, yaw * 180.0 / math.pi)

if __name__ == '__main__':
    try:
        # 創建OdomReader實例
        odom_reader = OdomReader()
        # 保持節點運行
        rospy.spin()
    except rospy.ROSInterruptException:
        pass