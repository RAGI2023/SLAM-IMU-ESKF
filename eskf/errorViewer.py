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

        self.eskf_buffer = []
        self.eskf_timestamps = []

        self.slam_positions = []
        self.eskf_positions = []
        self.slam_orientations = []
        self.eskf_orientations = []

        self.fig, self.axs = plt.subplots(3, 2, figsize=(12, 12))
        plt.ion()
        plt.show()
        self.last_time = time.time()

        self.eskf_odom_sub = self.create_subscription(Odometry, '/eskf_odom', self.eskf_odom_callback, 100)
        self.slam_odom_sub = self.create_subscription(Odometry, '/Odometry', self.slam_odom_callback, 10)

    def get_timestamp(self, msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def eskf_odom_callback(self, msg):
        t = self.get_timestamp(msg)
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.eskf_buffer.append(((pos.x, pos.y, pos.z), (ori.x, ori.y, ori.z, ori.w)))
        self.eskf_timestamps.append(t)

        # 控制缓存长度（可选）
        if len(self.eskf_buffer) > 2000:
            self.eskf_buffer.pop(0)
            self.eskf_timestamps.pop(0)

    def slam_odom_callback(self, msg):
        slam_time = self.get_timestamp(msg)
        slam_pos = msg.pose.pose.position
        slam_ori = msg.pose.pose.orientation

        # 找出时间上最接近的 ESKF 数据
        if not self.eskf_timestamps:
            return
        idx = np.argmin(np.abs(np.array(self.eskf_timestamps) - slam_time))

        # 丢弃相差太大的点（比如 0.1 秒以上）
        if abs(self.eskf_timestamps[idx] - slam_time) > 0.1:
            return

        eskf_pos, eskf_ori = self.eskf_buffer[idx]

        self.slam_positions.append((slam_pos.x, slam_pos.y, slam_pos.z))
        self.eskf_positions.append(eskf_pos)

        self.slam_orientations.append((slam_ori.x, slam_ori.y, slam_ori.z, slam_ori.w))
        self.eskf_orientations.append(eskf_ori)

    def quat_to_euler(self, q):
        r = R.from_quat(q)
        return r.as_euler('xyz', degrees=True)

    def update_plot(self):
        if len(self.slam_positions) < 2:
            return
        if time.time() - self.last_time < 0.2:
            return
        self.last_time = time.time()

        slam_pos = np.array(self.slam_positions)
        eskf_pos = np.array(self.eskf_positions)

        slam_euler = np.array([self.quat_to_euler(q) for q in self.slam_orientations])
        eskf_euler = np.array([self.quat_to_euler(q) for q in self.eskf_orientations])

        self.axs[0, 0].cla()
        self.axs[0, 0].plot(eskf_pos[:, 0], label='ESKF X')
        self.axs[0, 0].plot(slam_pos[:, 0], label='SLAM X', linestyle='--')
        self.axs[0, 0].set_title('X Position')
        self.axs[0, 0].legend()

        self.axs[0, 1].cla()
        self.axs[0, 1].plot(eskf_pos[:, 1], label='ESKF Y')
        self.axs[0, 1].plot(slam_pos[:, 1], label='SLAM Y', linestyle='--')
        self.axs[0, 1].set_title('Y Position')
        self.axs[0, 1].legend()

        self.axs[1, 0].cla()
        self.axs[1, 0].plot(eskf_pos[:, 2], label='ESKF Z')
        self.axs[1, 0].plot(slam_pos[:, 2], label='SLAM Z', linestyle='--')
        self.axs[1, 0].set_title('Z Position')
        self.axs[1, 0].legend()

        self.axs[1, 1].cla()
        self.axs[1, 1].plot(eskf_euler[:, 2], label='ESKF Yaw')
        self.axs[1, 1].plot(slam_euler[:, 2], label='SLAM Yaw', linestyle='--')
        self.axs[1, 1].set_title('Yaw Angle')
        self.axs[1, 1].legend()

        self.axs[2, 0].cla()
        self.axs[2, 0].plot(eskf_euler[:, 1], label='ESKF Pitch')
        self.axs[2, 0].plot(slam_euler[:, 1], label='SLAM Pitch', linestyle='--')
        self.axs[2, 0].set_title('Pitch Angle')
        self.axs[2, 0].legend()

        self.axs[2, 1].cla()
        self.axs[2, 1].plot(eskf_euler[:, 0], label='ESKF Roll')
        self.axs[2, 1].plot(slam_euler[:, 0], label='SLAM Roll', linestyle='--')
        self.axs[2, 1].set_title('Roll Angle')
        self.axs[2, 1].legend()

        self.fig.tight_layout()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            self.update_plot()

def main(args=None):
    rclpy.init(args=args)
    visualizer = OdometryVisualizer()
    visualizer.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
