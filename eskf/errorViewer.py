#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import numpy as np
import time

class OdometryVisualizer(Node):
    def __init__(self):
        super().__init__('odom_visualizer')

        # 订阅 Odom 数据
        self.eskf_odom_sub = self.create_subscription(
            Odometry,
            '/eskf_odom',  # 这里是 ESKF 里程计话题
            self.eskf_odom_callback,
            10
        )

        self.slam_odom_sub = self.create_subscription(
            Odometry,
            '/Odometry',  # 这里是 SLAM 里程计话题
            self.slam_odom_callback,
            10
        )

        # 存储 Odom 数据
        self.eskf_positions = []
        self.slam_positions = []
        self.eskf_orientations = []
        self.slam_orientations = []

        # 创建绘图窗口，布局为 3x2 子图
        self.fig, self.axs = plt.subplots(3, 2, figsize=(12, 12))

        plt.ion()  # 开启交互模式
        plt.show()  # 显示图形窗口

        # 初始化时间
        self.last_time = time.time()

    def eskf_odom_callback(self, msg):
        # 记录 ESKF 数据
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.eskf_positions.append((position.x, position.y, position.z))
        self.eskf_orientations.append((orientation.x, orientation.y, orientation.z, orientation.w))

    def slam_odom_callback(self, msg):
        # 记录 SLAM 数据
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.slam_positions.append((position.x, position.y, position.z))
        self.slam_orientations.append((orientation.x, orientation.y, orientation.z, orientation.w))

    def quat_to_euler(self, q):
        # 将四元数转换为欧拉角
        r = R.from_quat(q)
        return r.as_euler('xyz', degrees=True)  # 返回欧拉角，单位为度

    def update_plot(self):
        # 更新折线图
        if len(self.eskf_positions) > 1 and len(self.slam_positions) > 0:
            # 插值对齐 SLAM 和 ESKF 数据
            time_diff = time.time() - self.last_time
            if time_diff > 0.05:  # 只更新每 50 毫秒一次，避免更新过快
                self.last_time = time.time()

                # 位置误差
                eskf_pos = np.array(self.eskf_positions)
                slam_pos = np.array(self.slam_positions)

                # 插值（SLAM 频率较低，需要对齐）
                if len(slam_pos) > len(eskf_pos):
                    slam_pos = slam_pos[:len(eskf_pos)]
                elif len(slam_pos) < len(eskf_pos):
                    for i in range(len(eskf_pos) - len(slam_pos)):
                        slam_pos = np.vstack([slam_pos, slam_pos[-1]])

                # 姿态误差
                eskf_orient = np.array(self.eskf_orientations)
                slam_orient = np.array(self.slam_orientations)
                if len(slam_orient) > len(eskf_orient):
                    slam_orient = slam_orient[:len(eskf_orient)]
                elif len(slam_orient) < len(eskf_orient):
                    for i in range(len(eskf_orient) - len(slam_orient)):
                        slam_orient = np.vstack([slam_orient, slam_orient[-1]])

                # 计算误差
                pos_error = eskf_pos - slam_pos
                euler_eskf = np.array([self.quat_to_euler(o) for o in eskf_orient])
                euler_slam = np.array([self.quat_to_euler(o) for o in slam_orient])
                orientation_error = euler_eskf - euler_slam

                # 绘制位置误差 (X, Y, Z)
                self.axs[0, 0].cla()
                self.axs[0, 0].plot(pos_error[:, 0], label='X Error')
                self.axs[0, 0].set_title('X Error')
                self.axs[0, 0].legend()

                self.axs[0, 1].cla()
                self.axs[0, 1].plot(pos_error[:, 1], label='Y Error')
                self.axs[0, 1].set_title('Y Error')
                self.axs[0, 1].legend()

                self.axs[1, 0].cla()
                self.axs[1, 0].plot(pos_error[:, 2], label='Z Error')
                self.axs[1, 0].set_title('Z Error')
                self.axs[1, 0].legend()

                # 绘制姿态误差 (Yaw, Pitch, Roll)
                self.axs[1, 1].cla()
                self.axs[1, 1].plot(orientation_error[:, 2], label='Yaw Error')
                self.axs[1, 1].set_title('Yaw Error')
                self.axs[1, 1].legend()

                self.axs[2, 0].cla()
                self.axs[2, 0].plot(orientation_error[:, 1], label='Pitch Error')
                self.axs[2, 0].set_title('Pitch Error')
                self.axs[2, 0].legend()

                self.axs[2, 1].cla()
                self.axs[2, 1].plot(orientation_error[:, 0], label='Roll Error')
                self.axs[2, 1].set_title('Roll Error')
                self.axs[2, 1].legend()

                # 自动调整子图间距
                self.fig.tight_layout()

                # 刷新图形
                self.fig.canvas.draw()
                self.fig.canvas.flush_events()

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            self.update_plot()

def main(args=None):
    rclpy.init(args=args)
    visualizer = OdometryVisualizer()
    visualizer.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
