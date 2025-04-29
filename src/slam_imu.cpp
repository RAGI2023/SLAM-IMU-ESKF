#include "eskf/eskf.hpp"
#include "nav_msgs/Odometry.h"
#include "ros/console.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "ros/time.h"
#include "sensor_msgs/Imu.h"
class SLAM_IMU_Filter : public ErrorStateKalmanFilter {
 public:
  SLAM_IMU_Filter(const std::string &node_name, const std::string &imu_topic,
                  const std::string &odom_topic,
                  const std::string &out_topic_name, double gravity,
                  double pos_std, double ori_std, double gyr_noise,
                  double acc_noise, const std::string &filename,
                  bool init_ESKF = false)
      : ErrorStateKalmanFilter(gravity, pos_std, ori_std, gyr_noise, acc_noise,
                               filename),
        gravity_(-gravity) {
    imu_sub_ = nh_.subscribe<sensor_msgs::Imu>(
        imu_topic, 1,
        std::bind(&SLAM_IMU_Filter::imu_cb, this, std::placeholders::_1));
    odo_sub_ = nh_.subscribe<nav_msgs::Odometry>(
        odom_topic, 10,
        std::bind(&SLAM_IMU_Filter::odom_cb, this, std::placeholders::_1));
    out_pub_ = nh_.advertise<nav_msgs::Odometry>(out_topic_name, 1);
    if (init_ESKF) {
      initESKF();
    }
    ROS_INFO("%s Node start.", node_name.c_str());
  }

  SLAM_IMU_Filter(const std::string &node_name, const std::string &imu_topic,
                  const std::string &odom_topic,
                  const std::string &out_topic_name, double gravity,
                  double pos_noise, double vel_noise, double ori_noise,
                  double gyr_bias_noise, double acc_bias_noise,
                  double pos_vel_corr, double pos_std, double ori_std,
                  double gyr_noise, double acc_noise, bool init_ESKF = false)
      : ErrorStateKalmanFilter(gravity, pos_noise, vel_noise, ori_noise,
                               gyr_bias_noise, acc_bias_noise, pos_vel_corr,
                               pos_std, ori_std, gyr_noise, acc_noise),
        gravity_(-gravity) {
    // 构建imu odom订阅 高频定位发布
    imu_sub_ = nh_.subscribe<sensor_msgs::Imu>(
        imu_topic, 1,
        std::bind(&SLAM_IMU_Filter::imu_cb, this, std::placeholders::_1));
    odo_sub_ = nh_.subscribe<nav_msgs::Odometry>(
        odom_topic, 10,
        std::bind(&SLAM_IMU_Filter::odom_cb, this, std::placeholders::_1));
    out_pub_ = nh_.advertise<nav_msgs::Odometry>(out_topic_name, 1);
    if (init_ESKF) {
      initESKF();
    }
    ROS_INFO("%s Node start.", node_name.c_str());
  }

 private:
  ros::NodeHandle nh_;
  ros::Subscriber imu_sub_;
  ros::Subscriber odo_sub_;
  ros::Publisher out_pub_;
  double gravity_;  // 重力加速度大小，正数

  void initESKF(Eigen::Vector3d pose = Eigen::Vector3d::Zero(),
                Eigen::Quaterniond q = Eigen::Quaterniond::Identity(),
                Eigen::Vector3d vel = Eigen::Vector3d::Zero(),
                long long time = ros::Time::now().nsec) {
    Eigen::Matrix4d init_pose = Eigen::Matrix4d::Identity();
    init_pose.block<3, 3>(0, 0) = q.toRotationMatrix();
    init_pose.block<3, 1>(0, 3) = pose;
    Init(init_pose, vel, time);
    isInitialized = true;
    // this->Init(init_pose, vel, rclcpp::Clock().now().nanoseconds());
  }

  void imu_cb(const sensor_msgs::Imu::ConstPtr &msg) {
    if (this->isInitialized) {
      auto start_time = std::chrono::high_resolution_clock::now();
      Eigen::Vector3d pose, vel, angle_vel;
      Eigen::Quaterniond q;
      Eigen::Vector3d linear_acce(msg->linear_acceleration.x,
                                  msg->linear_acceleration.y,
                                  msg->linear_acceleration.z);
      std::cout << "Raw IMU acceleration: x=" << msg->linear_acceleration.x
                << ", y=" << msg->linear_acceleration.y
                << ", z=" << msg->linear_acceleration.z << std::endl;
      // livox mid 360 imu数据单位是g！
      linear_acce *= gravity_;
      linear_acce[2] = -linear_acce[2];
      std::cout << "linear accelerate ";
      std::cout << linear_acce << std::endl;
      bool success = Predict(
          linear_acce,
          Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y,
                          msg->angular_velocity.z),
          pose, vel, angle_vel, q, msg->header.stamp.nsec);
      if (success) {
        nav_msgs::Odometry pub_msg;
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

        out_pub_.publish(pub_msg);
      } else {
        std::cout << "Predict Skip" << std::endl;
      }
      auto end_time = std::chrono::high_resolution_clock::now();

      // Calculate the duration in milliseconds
      std::chrono::duration<double> duration = end_time - start_time;

      // Print the time taken to execute the function
      std::cout << "imu_cb execution time: " << duration.count() * 1000.0
                << " ms" << std::endl;
    }
  }
  void odom_cb(const nav_msgs::Odometry::ConstPtr &msg) {
    if (this->isInitialized) {
      Eigen::Vector3d pos(msg->pose.pose.position.x, msg->pose.pose.position.y,
                          msg->pose.pose.position.z);
      Eigen::Quaterniond q(
          msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
          msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
      correct(pos, q);
    } else {
      Eigen::Vector3d pose(msg->pose.pose.position.x, msg->pose.pose.position.y,
                           msg->pose.pose.position.z);
      Eigen::Quaterniond q(
          msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
          msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
      long long time = msg->header.stamp.nsec;
      initESKF(pose, q, Eigen::Vector3d::Zero(), time);
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "slam_imu");
  SLAM_IMU_Filter eskf("eskf_node",          // 节点名
                       "/livox/imu",         // imu话题
                       "/Odometry",          // slam 里程计话题
                       "/eskf_odom",         // 输出话题
                       -9.8015,              // 重力
                       0.1,                  // 位置噪声
                       0.1,                  // 速度噪声
                       0.1,                  // 姿态噪声
                       0.0003158085227 * 3,  // 陀螺仪偏置噪声
                       0.001117221 * 3,      // 加速度计偏置噪声
                       0.03,                 // 速度位置协方差
                       0.006511,             // slam位置测量标准差
                       1.179523e-03,         // slam姿态测量标准差
                       0.00143,              // 陀螺仪过程噪声
                       0.0386,               // 加速度计过程噪声
                       false  // 是否使用当前时间初始化
  );

  ros::spin();
  return 0;
}