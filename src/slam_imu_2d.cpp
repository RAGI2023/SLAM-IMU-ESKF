#include <signal.h>  // 处理 Ctrl+C

#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <ostream>
#include <rclcpp/clock.hpp>
#include <rclcpp/create_subscription.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/detail/imu__struct.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "eskf/eskf2D.hpp"

class SLAM_IMU_Filter2D : public ErrorStateKalmanFilter2D, public rclcpp::Node {
 public:
  SLAM_IMU_Filter2D(const std::string &node_name, const std::string &imu_topic,
                    const std::string &odom_topic,
                    const std::string &out_topic_name, double gravity,
                    double pos_noise, double vel_noise, double ori_noise,
                    double gyr_bias_noise, double acc_bias_noise,
                    double pos_vel_corr, double pos_std, double ori_std,
                    double gyr_noise, double acc_noise, bool init_ESKF = false)
      : ErrorStateKalmanFilter2D(gravity, pos_noise, vel_noise, ori_noise,
                                 gyr_bias_noise, acc_bias_noise, pos_vel_corr,
                                 pos_std, ori_std, gyr_noise, acc_noise),
        Node(node_name),
        gravity_(gravity) {
    // 构建imu odom订阅 高频定位发布
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, 1,
        std::bind(&SLAM_IMU_Filter2D::imu_cb, this, std::placeholders::_1));
    odo_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 10,
        std::bind(&SLAM_IMU_Filter2D::odom_cb, this, std::placeholders::_1));
    out_pub_ =
        this->create_publisher<nav_msgs::msg::Odometry>(out_topic_name, 1);
    if (init_ESKF) {
      initESKF();
    }
    RCLCPP_INFO(this->get_logger(), "%s Node start.", node_name.c_str());
  }

 private:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odo_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr out_pub_;
  double gravity_;  // 重力加速度大小，正数

  void initESKF(
      Eigen::Vector3d pose = Eigen::Vector3d::Zero(),
      geometry_msgs::msg::Quaternion q_msg = geometry_msgs::msg::Quaternion(),
      Eigen::Vector2d vel = Eigen::Vector2d::Zero(), long long time = 0) {
    // 设置默认四元数 (单位四元数)
    if (q_msg.w == 0 && q_msg.x == 0 && q_msg.y == 0 && q_msg.z == 0) {
      q_msg.w = 1.0;
      q_msg.x = 0.0;
      q_msg.y = 0.0;
      q_msg.z = 0.0;
    }

    // 从四元数提取yaw角
    double yaw = quaternionToYaw(q_msg);
    Eigen::Vector3d init_pose_2d(pose.x(), pose.y(), yaw);

    if (time == 0) {
      time = this->get_clock()->now().nanoseconds();
    }

    Init(init_pose_2d, vel, time);
    isInitialized = true;
    RCLCPP_INFO(this->get_logger(),
                "ESKF initialized at pose: [%.3f, %.3f, %.3f]", pose.x(),
                pose.y(), yaw);
  }

  // 四元数转换为yaw角的辅助函数 (使用数学公式，不依赖tf2)
  double quaternionToYaw(const geometry_msgs::msg::Quaternion &q) {
    // 使用四元数到欧拉角转换公式
    // yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  // yaw角转换为四元数的辅助函数 (使用数学公式，不依赖tf2)
  geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) {
    geometry_msgs::msg::Quaternion q;
    // 绕z轴旋转的四元数: q = [0, 0, sin(yaw/2), cos(yaw/2)]
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw / 2.0);
    q.w = std::cos(yaw / 2.0);
    return q;
  }

  void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg) {
    if (this->isInitialized) {
      auto start_time = std::chrono::high_resolution_clock::now();

      Eigen::Vector2d pos, vel;
      double angle_vel, yaw;

      // 提取2D加速度数据 (只使用x,y轴)
      Eigen::Vector2d linear_acce(msg->linear_acceleration.x,
                                  msg->linear_acceleration.y);

      RCLCPP_INFO_STREAM(
          this->get_logger(),
          "Raw IMU acceleration: x=" << msg->linear_acceleration.x
                                     << ", y=" << msg->linear_acceleration.y);

      // livox mid 360 imu数据单位是g！需要转换为m/s²
      linear_acce *= std::abs(gravity_);

      // 提取z轴角速度
      double angular_vel_z = msg->angular_velocity.z;

      RCLCPP_INFO_STREAM(this->get_logger(), "2D linear accelerate: ["
                                                 << linear_acce.transpose()
                                                 << "]");
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Angular velocity z: " << angular_vel_z);

      bool success = Predict(linear_acce, angular_vel_z, pos, vel, angle_vel,
                             yaw, msg->header.stamp.nanosec);

      if (success) {
        nav_msgs::msg::Odometry pub_msg;
        pub_msg.header = msg->header;

        // 位置 (2D -> 3D, z=0)
        pub_msg.pose.pose.position.x = pos.x();
        pub_msg.pose.pose.position.y = pos.y();
        pub_msg.pose.pose.position.z = 0.0;  // 2D中z固定为0

        // 姿态 (yaw -> 四元数)
        pub_msg.pose.pose.orientation = yawToQuaternion(yaw);

        // 速度 (2D -> 3D, vz=0)
        pub_msg.twist.twist.linear.x = vel.x();
        pub_msg.twist.twist.linear.y = vel.y();
        pub_msg.twist.twist.linear.z = 0.0;  // 2D中vz固定为0

        // 角速度 (只有z轴)
        pub_msg.twist.twist.angular.x = 0.0;
        pub_msg.twist.twist.angular.y = 0.0;
        pub_msg.twist.twist.angular.z = angle_vel;

        out_pub_->publish(pub_msg);
        RCLCPP_INFO_STREAM(this->get_logger(), "Predict Send");
      } else {
        RCLCPP_WARN_STREAM(this->get_logger(), "Predict Skip");
      }

      auto end_time = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> duration = end_time - start_time;
      RCLCPP_INFO_STREAM(
          this->get_logger(),
          "imu_cb execution time: " << duration.count() * 1000.0 << " ms");
    }
  }

  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (this->isInitialized) {
      // 提取2D位置
      Eigen::Vector2d pos(msg->pose.pose.position.x, msg->pose.pose.position.y);

      // 提取yaw角
      double yaw = quaternionToYaw(msg->pose.pose.orientation);

      correct(pos, yaw);

      RCLCPP_INFO(this->get_logger(),
                  "SLAM correction: pos=[%.3f, %.3f], yaw=%.3f", pos.x(),
                  pos.y(), yaw);
    } else {
      // 初始化ESKF
      Eigen::Vector3d pose_3d(msg->pose.pose.position.x,
                              msg->pose.pose.position.y,
                              msg->pose.pose.position.z);

      // 提取2D速度
      Eigen::Vector2d vel_2d(msg->twist.twist.linear.x,
                             msg->twist.twist.linear.y);

      long long time = msg->header.stamp.nanosec;
      initESKF(pose_3d, msg->pose.pose.orientation, vel_2d, time);
    }
  }
};

std::shared_ptr<SLAM_IMU_Filter2D> node_ptr = nullptr;

void signal_handler(int signal) {
  if (signal == SIGINT) {
    RCLCPP_INFO(rclcpp::get_logger("eskf_2d_node"),
                "Caught Ctrl+C (SIGINT), start shutdown process...");

    if (node_ptr) {
      node_ptr->printParameters(
          "/home/yin/eskf_ws/out_2d.txt");  // 调用你自定义的保存函数
    }

    rclcpp::shutdown();  // 通知rclcpp退出
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  node_ptr =
      std::make_shared<SLAM_IMU_Filter2D>("eskf_2d_node",  // 节点名
                                          "/livox/imu",    // imu话题
                                          "/Odometry",  // slam 里程计话题
                                          "/eskf_2d_odom",  // 输出话题
                                          9.8015,  // 重力 (注意这里改为正数)
                                          0.01,                 // 位置噪声
                                          0.01,                 // 速度噪声
                                          0.01,                 // 姿态噪声
                                          0.0003158085227 * 3,  //陀螺仪偏置噪声
                                          0.001117221 * 3,  //加速度计偏置噪声
                                          0.03,       //速度位置协方差
                                          0.0006511,  // slam位置测量标准差
                                          1.179523e-03,  // slam姿态测量标准差
                                          0.0143 * 10,  //陀螺仪过程噪声
                                          0.0386 * 10,  //  加速度计过程噪声
                                          false  // 是否使用当前时间初始化
      );

  signal(SIGINT, signal_handler);  // 处理Ctrl+C信号
  rclcpp::spin(node_ptr);
  rclcpp::shutdown();
  return 0;
}