#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import numpy as np
import time
import math

class OdometryVisualizer2D(Node):
    def __init__(self):
        super().__init__('odom_visualizer_2d')

        self.eskf_buffer = []
        self.eskf_timestamps = []

        self.slam_positions = []
        self.eskf_positions = []
        self.slam_orientations = []
        self.eskf_orientations = []

        # 创建2D可视化布局：2x2 + 1个轨迹图
        self.fig, self.axs = plt.subplots(2, 3, figsize=(15, 10))
        plt.ion()
        plt.show()
        self.last_time = time.time()

        # 订阅2D ESKF输出和SLAM输出
        self.eskf_odom_sub = self.create_subscription(Odometry, '/eskf_2d_odom', self.eskf_odom_callback, 100)
        self.slam_odom_sub = self.create_subscription(Odometry, '/Odometry', self.slam_odom_callback, 10)
        
        self.get_logger().info("2D Odometry Visualizer started")

    def get_timestamp(self, msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def eskf_odom_callback(self, msg):
        self.get_logger().info("Received ESKF data")
        t = self.get_timestamp(msg)
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        
        # 只存储2D位置和yaw角
        self.eskf_buffer.append(((pos.x, pos.y), (ori.x, ori.y, ori.z, ori.w)))
        self.eskf_timestamps.append(t)

        # 控制缓存长度
        if len(self.eskf_buffer) > 2000:
            self.eskf_buffer.pop(0)
            self.eskf_timestamps.pop(0)

    def slam_odom_callback(self, msg):
        self.get_logger().info("Received SLAM data")
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

        # 只存储2D位置数据
        self.slam_positions.append((slam_pos.x, slam_pos.y))
        self.eskf_positions.append(eskf_pos)

        self.slam_orientations.append((slam_ori.x, slam_ori.y, slam_ori.z, slam_ori.w))
        self.eskf_orientations.append(eskf_ori)

    def quat_to_yaw(self, q):
        """将四元数转换为yaw角（度）"""
        # 使用数学公式直接计算yaw角
        siny_cosp = 2 * (q[3] * q[2] + q[0] * q[1])  # q = [x, y, z, w]
        cosy_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2])
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        return math.degrees(yaw_rad)

    def calculate_errors(self, slam_data, eskf_data):
        """计算误差统计"""
        if len(slam_data) < 2:
            return None
        
        errors = np.array(eskf_data) - np.array(slam_data)
        return {
            'mean': np.mean(errors, axis=0),
            'std': np.std(errors, axis=0),
            'rmse': np.sqrt(np.mean(errors**2, axis=0))
        }

    def update_plot(self):
        if len(self.slam_positions) < 2:
            return
        if time.time() - self.last_time < 0.2:
            return
        self.last_time = time.time()

        slam_pos = np.array(self.slam_positions)
        eskf_pos = np.array(self.eskf_positions)

        # 计算yaw角
        slam_yaw = np.array([self.quat_to_yaw(q) for q in self.slam_orientations])
        eskf_yaw = np.array([self.quat_to_yaw(q) for q in self.eskf_orientations])

        # X位置对比
        self.axs[0, 0].cla()
        self.axs[0, 0].plot(eskf_pos[:, 0], label='ESKF X', color='blue', linewidth=2)
        self.axs[0, 0].plot(slam_pos[:, 0], label='SLAM X', color='red', linestyle='--', linewidth=2)
        self.axs[0, 0].set_title('X Position Comparison')
        self.axs[0, 0].set_xlabel('Time Steps')
        self.axs[0, 0].set_ylabel('X Position (m)')
        self.axs[0, 0].legend()
        self.axs[0, 0].grid(True, alpha=0.3)

        # Y位置对比
        self.axs[0, 1].cla()
        self.axs[0, 1].plot(eskf_pos[:, 1], label='ESKF Y', color='blue', linewidth=2)
        self.axs[0, 1].plot(slam_pos[:, 1], label='SLAM Y', color='red', linestyle='--', linewidth=2)
        self.axs[0, 1].set_title('Y Position Comparison')
        self.axs[0, 1].set_xlabel('Time Steps')
        self.axs[0, 1].set_ylabel('Y Position (m)')
        self.axs[0, 1].legend()
        self.axs[0, 1].grid(True, alpha=0.3)

        # Yaw角对比
        self.axs[0, 2].cla()
        self.axs[0, 2].plot(eskf_yaw, label='ESKF Yaw', color='blue', linewidth=2)
        self.axs[0, 2].plot(slam_yaw, label='SLAM Yaw', color='red', linestyle='--', linewidth=2)
        self.axs[0, 2].set_title('Yaw Angle Comparison')
        self.axs[0, 2].set_xlabel('Time Steps')
        self.axs[0, 2].set_ylabel('Yaw Angle (degrees)')
        self.axs[0, 2].legend()
        self.axs[0, 2].grid(True, alpha=0.3)

        # 2D轨迹对比
        self.axs[1, 0].cla()
        self.axs[1, 0].plot(eskf_pos[:, 0], eskf_pos[:, 1], label='ESKF Trajectory', 
                           color='blue', linewidth=2, marker='o', markersize=2)
        self.axs[1, 0].plot(slam_pos[:, 0], slam_pos[:, 1], label='SLAM Trajectory', 
                           color='red', linewidth=2, linestyle='--', marker='s', markersize=2)
        self.axs[1, 0].set_title('2D Trajectory Comparison')
        self.axs[1, 0].set_xlabel('X Position (m)')
        self.axs[1, 0].set_ylabel('Y Position (m)')
        self.axs[1, 0].legend()
        self.axs[1, 0].grid(True, alpha=0.3)
        self.axs[1, 0].axis('equal')  # 保持比例

        # 位置误差
        self.axs[1, 1].cla()
        pos_errors = eskf_pos - slam_pos
        self.axs[1, 1].plot(pos_errors[:, 0], label='X Error', color='green', linewidth=2)
        self.axs[1, 1].plot(pos_errors[:, 1], label='Y Error', color='orange', linewidth=2)
        self.axs[1, 1].set_title('Position Errors')
        self.axs[1, 1].set_xlabel('Time Steps')
        self.axs[1, 1].set_ylabel('Position Error (m)')
        self.axs[1, 1].legend()
        self.axs[1, 1].grid(True, alpha=0.3)

        # 角度误差和统计信息
        self.axs[1, 2].cla()
        yaw_errors = eskf_yaw - slam_yaw
        # 处理角度跨越问题
        yaw_errors = np.where(yaw_errors > 180, yaw_errors - 360, yaw_errors)
        yaw_errors = np.where(yaw_errors < -180, yaw_errors + 360, yaw_errors)
        
        self.axs[1, 2].plot(yaw_errors, label='Yaw Error', color='purple', linewidth=2)
        self.axs[1, 2].set_title('Yaw Angle Error')
        self.axs[1, 2].set_xlabel('Time Steps')
        self.axs[1, 2].set_ylabel('Yaw Error (degrees)')
        self.axs[1, 2].legend()
        self.axs[1, 2].grid(True, alpha=0.3)

        # 添加统计信息文本
        pos_stats = self.calculate_errors(slam_pos, eskf_pos)
        if pos_stats:
            stats_text = f"Position RMSE:\nX: {pos_stats['rmse'][0]:.4f}m\nY: {pos_stats['rmse'][1]:.4f}m\n"
            stats_text += f"Yaw RMSE: {np.sqrt(np.mean(yaw_errors**2)):.2f}°"
            self.axs[1, 2].text(0.02, 0.98, stats_text, transform=self.axs[1, 2].transAxes,
                               verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

        self.fig.suptitle('2D ESKF vs SLAM Odometry Comparison', fontsize=16, fontweight='bold')
        self.fig.tight_layout()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def run(self):
        self.get_logger().info("Starting visualization loop...")
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            self.update_plot()

def main(args=None):
    rclpy.init(args=args)
    visualizer = OdometryVisualizer2D()
    try:
        visualizer.run()
    except KeyboardInterrupt:
        print("\nShutting down visualizer...")
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()
        plt.close('all')

if __name__ == '__main__':
    main()