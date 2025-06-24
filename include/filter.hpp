#pragma once

#include <Eigen/Dense>
#include <deque>
#include <nav_msgs/msg/odometry.hpp>
class MovingAveFilter {
 private:
  double window_size_;  // Size of the moving average window
  std::deque<nav_msgs::msg::Odometry>
      buffer_;  // Deque to store the last N odometry messages
 public:
  MovingAveFilter(double window_size) { window_size_ = window_size; }

  nav_msgs::msg::Odometry filter(const nav_msgs::msg::Odometry& odom) {
    // Add the new odometry message to the buffer
    buffer_.push_back(odom);

    // If the buffer exceeds the window size, remove the oldest message
    if (buffer_.size() > window_size_) {
      buffer_.pop_front();
    }

    // Calculate the moving average
    nav_msgs::msg::Odometry avg_odom = odom;
    for (const auto& msg : buffer_) {
      avg_odom.pose.pose.position.x += msg.pose.pose.position.x;
      avg_odom.pose.pose.position.y += msg.pose.pose.position.y;
      avg_odom.pose.pose.position.z += msg.pose.pose.position.z;
      avg_odom.twist.twist.linear.x += msg.twist.twist.linear.x;
      avg_odom.twist.twist.linear.y += msg.twist.twist.linear.y;
      avg_odom.twist.twist.linear.z += msg.twist.twist.linear.z;
      avg_odom.pose.pose.orientation.w += msg.pose.pose.orientation.w;
      avg_odom.pose.pose.orientation.x += msg.pose.pose.orientation.x;
      avg_odom.pose.pose.orientation.y += msg.pose.pose.orientation.y;
      avg_odom.pose.pose.orientation.z += msg.pose.pose.orientation.z;
    }

    // Average the values
    double count = static_cast<double>(buffer_.size());
    avg_odom.pose.pose.position.x /= count;
    avg_odom.pose.pose.position.y /= count;
    avg_odom.pose.pose.position.z /= count;
    avg_odom.twist.twist.linear.x /= count;
    avg_odom.twist.twist.linear.y /= count;
    avg_odom.twist.twist.linear.z /= count;
    Eigen::Quaterniond tmp_q(
        avg_odom.pose.pose.orientation.w, avg_odom.pose.pose.orientation.x,
        avg_odom.pose.pose.orientation.y, avg_odom.pose.pose.orientation.z);
    tmp_q.normalize();
    avg_odom.pose.pose.orientation.w = tmp_q.w();
    avg_odom.pose.pose.orientation.x = tmp_q.x();
    avg_odom.pose.pose.orientation.y = tmp_q.y();
    avg_odom.pose.pose.orientation.z = tmp_q.z();

    return avg_odom;
  }
};

class MedianFilter {
 private:
  size_t window_size_;
  std::deque<nav_msgs::msg::Odometry> buffer_;

  double getMedian(std::vector<double> values) {
    std::sort(values.begin(), values.end());
    size_t n = values.size();
    if (n % 2 == 0) {
      return (values[n / 2 - 1] + values[n / 2]) / 2.0;
    } else {
      return values[n / 2];
    }
  }

  // 四元数中值需要特殊处理，这里用简单的归一化平均
  geometry_msgs::msg::Quaternion getMedianQuaternion(
      const std::deque<nav_msgs::msg::Odometry>& buffer) {
    // 对于四元数，中值滤波比较复杂，这里使用平均后归一化的方法
    geometry_msgs::msg::Quaternion avg_quat;
    avg_quat.w = 0;
    avg_quat.x = 0;
    avg_quat.y = 0;
    avg_quat.z = 0;

    for (const auto& msg : buffer) {
      avg_quat.w += msg.pose.pose.orientation.w;
      avg_quat.x += msg.pose.pose.orientation.x;
      avg_quat.y += msg.pose.pose.orientation.y;
      avg_quat.z += msg.pose.pose.orientation.z;
    }

    double count = static_cast<double>(buffer.size());
    Eigen::Quaterniond tmp_q(avg_quat.w / count, avg_quat.x / count,
                             avg_quat.y / count, avg_quat.z / count);
    tmp_q.normalize();

    avg_quat.w = tmp_q.w();
    avg_quat.x = tmp_q.x();
    avg_quat.y = tmp_q.y();
    avg_quat.z = tmp_q.z();

    return avg_quat;
  }

 public:
  MedianFilter(size_t window_size) : window_size_(window_size) {}

  nav_msgs::msg::Odometry filter(const nav_msgs::msg::Odometry& odom) {
    // Add the new odometry message to the buffer
    buffer_.push_back(odom);

    // If the buffer exceeds the window size, remove the oldest message
    if (buffer_.size() > window_size_) {
      buffer_.pop_front();
    }

    // Extract values for median calculation
    std::vector<double> pos_x, pos_y, pos_z;
    std::vector<double> vel_x, vel_y, vel_z;

    for (const auto& msg : buffer_) {
      pos_x.push_back(msg.pose.pose.position.x);
      pos_y.push_back(msg.pose.pose.position.y);
      pos_z.push_back(msg.pose.pose.position.z);
      vel_x.push_back(msg.twist.twist.linear.x);
      vel_y.push_back(msg.twist.twist.linear.y);
      vel_z.push_back(msg.twist.twist.linear.z);
    }

    // Calculate median values
    nav_msgs::msg::Odometry median_odom = odom;
    median_odom.pose.pose.position.x = getMedian(pos_x);
    median_odom.pose.pose.position.y = getMedian(pos_y);
    median_odom.pose.pose.position.z = getMedian(pos_z);
    median_odom.twist.twist.linear.x = getMedian(vel_x);
    median_odom.twist.twist.linear.y = getMedian(vel_y);
    median_odom.twist.twist.linear.z = getMedian(vel_z);

    // Handle quaternion (use average method for simplicity)
    median_odom.pose.pose.orientation = getMedianQuaternion(buffer_);

    return median_odom;
  }
};

class LowPassFilter {
 private:
  double alpha_;
  nav_msgs::msg::Odometry prev_output_;
  bool initialized_;

  // 四元数球面线性插值
  geometry_msgs::msg::Quaternion slerpQuaternion(
      const geometry_msgs::msg::Quaternion& q1,
      const geometry_msgs::msg::Quaternion& q2, double t) {
    Eigen::Quaterniond quat1(q1.w, q1.x, q1.y, q1.z);
    Eigen::Quaterniond quat2(q2.w, q2.x, q2.y, q2.z);

    Eigen::Quaterniond result = quat1.slerp(t, quat2);

    geometry_msgs::msg::Quaternion output;
    output.w = result.w();
    output.x = result.x();
    output.y = result.y();
    output.z = result.z();

    return output;
  }

 public:
  // 构造函数：可以传入截止频率和采样频率
  LowPassFilter(double cutoff_freq, double sample_rate) : initialized_(false) {
    double rc = 1.0 / (2.0 * M_PI * cutoff_freq);
    double dt = 1.0 / sample_rate;
    alpha_ = dt / (rc + dt);
  }

  // 或者直接传入alpha值
  LowPassFilter(double alpha) : alpha_(alpha), initialized_(false) {}

  nav_msgs::msg::Odometry filter(const nav_msgs::msg::Odometry& odom) {
    if (!initialized_) {
      prev_output_ = odom;
      initialized_ = true;
      return odom;
    }

    nav_msgs::msg::Odometry output = odom;

    // 位置滤波
    output.pose.pose.position.x =
        alpha_ * odom.pose.pose.position.x +
        (1 - alpha_) * prev_output_.pose.pose.position.x;
    output.pose.pose.position.y =
        alpha_ * odom.pose.pose.position.y +
        (1 - alpha_) * prev_output_.pose.pose.position.y;
    output.pose.pose.position.z =
        alpha_ * odom.pose.pose.position.z +
        (1 - alpha_) * prev_output_.pose.pose.position.z;

    // 速度滤波
    output.twist.twist.linear.x =
        alpha_ * odom.twist.twist.linear.x +
        (1 - alpha_) * prev_output_.twist.twist.linear.x;
    output.twist.twist.linear.y =
        alpha_ * odom.twist.twist.linear.y +
        (1 - alpha_) * prev_output_.twist.twist.linear.y;
    output.twist.twist.linear.z =
        alpha_ * odom.twist.twist.linear.z +
        (1 - alpha_) * prev_output_.twist.twist.linear.z;

    // 角速度滤波
    output.twist.twist.angular.x =
        alpha_ * odom.twist.twist.angular.x +
        (1 - alpha_) * prev_output_.twist.twist.angular.x;
    output.twist.twist.angular.y =
        alpha_ * odom.twist.twist.angular.y +
        (1 - alpha_) * prev_output_.twist.twist.angular.y;
    output.twist.twist.angular.z =
        alpha_ * odom.twist.twist.angular.z +
        (1 - alpha_) * prev_output_.twist.twist.angular.z;

    // 四元数滤波（使用球面线性插值）
    output.pose.pose.orientation = slerpQuaternion(
        prev_output_.pose.pose.orientation, odom.pose.pose.orientation, alpha_);

    prev_output_ = output;
    return output;
  }

  // 重置滤波器
  void reset() { initialized_ = false; }
};