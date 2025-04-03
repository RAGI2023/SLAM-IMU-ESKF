#include <functional>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/create_subscription.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/detail/imu__struct.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "eskf/eskf.hpp"

class SLAM_IMU_Filter : public ErrorStateKalmanFilter, public rclcpp::Node {
 public:
  SLAM_IMU_Filter(const std::string &node_name, const std::string &imu_topic,
                  const std::string &odom_topic,
                  const std::string &out_topic_name, double gravity,
                  double pos_noise, double vel_noise, double ori_noise,
                  double gyr_bias_noise, double acc_bias_noise, double pos_std,
                  double ori_std, double gyr_noise, double acc_noise)
      : ErrorStateKalmanFilter(gravity, pos_noise, vel_noise, ori_noise,
                               gyr_bias_noise, acc_bias_noise, pos_std, ori_std,
                               gyr_noise, acc_noise),
        Node(node_name) {
    // 构建imu odom订阅 高频定位发布
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, 100,
        std::bind(&SLAM_IMU_Filter::imu_cb, this, std::placeholders::_1));
    odo_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 10,
        std::bind(&SLAM_IMU_Filter::odom_cb, this, std::placeholders::_1));
    out_pub_ =
        this->create_publisher<nav_msgs::msg::Odometry>(out_topic_name, 1);
    initESKF();
    RCLCPP_INFO(this->get_logger(), "%s Node start.", node_name.c_str());
  }

 private:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odo_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr out_pub_;

  void initESKF() {
    // 构建单位四元数，无旋转
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    // 初始化初始位姿，无旋转，处于原点，静止
    Eigen::Matrix3d init_pose = Eigen::Matrix4d::Identity();
    init_pose.block<3, 3>(0, 0) = q.toRotationMatrix();
    init_pose.block<3, 1>(0, 3) = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d init_vel = Eigen::Matrix3d::Zero();
    // 获取时间戳
    this->Init(init_pose, init_vel, rclcpp::Clock().now().nanoseconds());
  }
  void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg) {
    if (this->isInitialized) {
      Eigen::Vector3d pose, vel, angle_vel;
      Eigen::Quaterniond q;
      bool success = this->Predict(
          Eigen::Vector3d(msg->linear_acceleration.x,
                          msg->linear_acceleration.y,
                          msg->linear_acceleration.z),
          Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y,
                          msg->angular_velocity.z),
          pose, vel, angle_vel, q, msg->header.stamp.nanosec);
      if (success) {
        nav_msgs::msg::Odometry pub_msg;
        pub_msg.header = msg->header;
        pub_msg.pose.pose.position.x = pose.x();
        pub_msg.pose.pose.position.y = pose.y();
        pub_msg.pose.pose.position.z = pose.z();

        pub_msg.pose.pose.orientation.w = q.w();
        pub_msg.pose.pose.orientation.x = q.x();
        pub_msg.pose.pose.orientation.y = q.y();
        pub_msg.pose.pose.orientation.z = q.z();

        pub_msg.twist.twist.linear.x = vel.x();
        pub_msg.twist.twist.linear.y = vel.y();
        pub_msg.twist.twist.linear.z = vel.z();

        pub_msg.twist.twist.angular.x = angle_vel.x();
        pub_msg.twist.twist.angular.y = angle_vel.y();
        pub_msg.twist.twist.angular.z = angle_vel.z();

        out_pub_->publish(pub_msg);
      }
    }
  }
  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (this->isInitialized) {
      Eigen::Vector3d pose(msg->pose.pose.position.x, msg->pose.pose.position.y,
                           msg->pose.pose.position.z);
      Eigen::Quaterniond q(
          msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
          msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
      this->correct(pose, q);
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SLAM_IMU_Filter>("");
  // rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}