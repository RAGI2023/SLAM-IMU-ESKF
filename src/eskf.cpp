#include "eskf/eskf.hpp"

#include <iostream>
#include <mutex>

/**
 * @brief Construct a new Error State Kalman Filter:: Error State Kalman Filter
 * object
 *
 * @param gravity 重力加速度
 * @param pos_noise 位置噪声
 * @param vel_noise 速度噪声
 * @param ori_noise 姿态噪声
 * @param gyr_bias_noise 陀螺仪偏置噪声
 * @param acc_bias_noise 加速度计偏置噪声
 * @param pos_std slam位置测量标准差
 * @param ori_std slam姿态测量标准差
 * @param gyr_noise 陀螺仪过程噪声
 * @param acc_noise 加速度计过程噪声
 */
ErrorStateKalmanFilter::ErrorStateKalmanFilter(
    double gravity, double pos_noise, double vel_noise, double ori_noise,
    double gyr_bias_noise, double acc_bias_noise, double pos_std,
    double ori_std, double gyr_noise, double acc_noise) {
  m_g = Eigen::Vector3d(0.0, 0.0, gravity);
  // 初始化协方差矩阵，协方差设为0，设置方差
  m_P.block<3, 3>(INDEX_STATE_POSI, INDEX_STATE_POSI) =
      Eigen::Matrix3d::Identity() * pos_noise * pos_noise;
  m_P.block<3, 3>(INDEX_STATE_VEL, INDEX_STATE_VEL) =
      Eigen::Matrix3d::Identity() * vel_noise * vel_noise;
  m_P.block<3, 3>(INDEX_STATE_ORI, INDEX_STATE_ORI) =
      Eigen::Matrix3d::Identity() * ori_noise * ori_noise;
  m_P.block<3, 3>(INDEX_STATE_GYRO_BIAS, INDEX_STATE_GYRO_BIAS) =
      Eigen::Matrix3d::Identity() * gyr_bias_noise * gyr_bias_noise;
  m_P.block<3, 3>(INDEX_STATE_ACC_BIAS, INDEX_STATE_ACC_BIAS) =
      Eigen::Matrix3d::Identity() * acc_bias_noise * acc_bias_noise;
  // 初始化测量噪声矩阵
  m_R.block<3, 3>(0, 0) = Eigen::Matrix3d::Ones() * pos_std * pos_std;
  m_R.block<3, 3>(3, 3) = Eigen::Matrix3d::Ones() * ori_std * ori_std;
  // 初始化过程噪声协方差矩阵
  m_Q.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * gyr_noise * gyr_noise;
  m_Q.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * acc_noise * acc_noise;
  // 初始真值为0
  m_X = Eigen::Matrix<double, DIM_STATE, 1>::Zero();
  // 状态转移矩阵赋为0
  m_F = Eigen::Matrix<double, DIM_STATE, DIM_STATE>::Zero();
  // 测量噪声的变换矩阵
  m_C = Eigen::Matrix<double, DIM_MEASUREMENT_NOISE,
                      DIM_MEASUREMENT_NOISE>::Identity();
  // 从状态中提取位置和姿态信息，将X投影到测量空间Y
  m_G.block<3, 3>(INDEX_MEASUREMENT_POSI, INDEX_MEASUREMENT_POSI) =
      Eigen::Matrix3d::Identity();
  m_G.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity();
}

void ErrorStateKalmanFilter::Init(Eigen::Matrix4d initPose,
                                  Eigen::Vector3d initVel, long long tc) {
  m_pose = initPose;
  m_velocity = initVel;
  // 记录时间戳
  m_last_imu_tc = tc;
  // 偏置置0
  m_last_unbias_acc = Eigen::Vector3d::Zero();
  m_last_unbias_gyr = Eigen::Vector3d::Zero();
  isInitialized = true;
  std::cout << "init pose: \n" << m_pose << std::endl;
}

bool ErrorStateKalmanFilter::Predict(Eigen::Vector3d imu_acc,
                                     Eigen::Vector3d imu_gyr,
                                     Eigen::Vector3d& pos, Eigen::Vector3d& vel,
                                     Eigen::Vector3d& angle_vel,
                                     Eigen::Quaterniond& q, long long tc) {
  // 计算时间差
  double delta_t = (tc - m_last_imu_tc) / 1000000000.0;
  if (delta_t < 0.005 || delta_t > 0.015) {
    m_last_imu_tc = tc;
    return false;
  }

  std::lock_guard<std::mutex> lock(m_mtx);

  std::cout << "delta sec: " << delta_t << std::endl;
  // 加速度计数据去除偏置
  Eigen::Vector3d unbias_gyr = imu_gyr - m_gyro_bias;
  // 两帧之间的旋转角度
  Eigen::Vector3d phi = (unbias_gyr + m_last_unbias_gyr) / 2.0 * delta_t;
  double phi_norm = phi.norm();
  // 更新旋转矩阵
  Eigen::Matrix3d skew_sym_m_phi;
  skew_sym_m_phi << 0.0, -phi[2], phi[1], phi[2], 0.0, -phi[0], -phi[1], phi[0],
      0.0;
  Eigen::Matrix3d R_i0_i1 = Eigen::Matrix3d::Identity() +
                            sinf64(phi_norm) / phi_norm * skew_sym_m_phi +
                            (1 - cosf64(phi_norm)) / (phi_norm * phi_norm) *
                                skew_sym_m_phi * skew_sym_m_phi;
  // 更新姿态
  Eigen::Matrix3d last_pose = m_pose.block<3, 3>(0, 0);
  m_pose.block<3, 3>(0, 0) = last_pose * R_i0_i1;
  // 去除加速度偏置
  Eigen::Vector3d unbias_acc = imu_acc - m_accel_bias - m_g;
  // 去除加速度数据偏置
  Eigen::Vector3d last_vel = m_velocity;
  // 更新速度要先把imu数据变换到世界坐标系
  m_velocity =
      last_vel +
      ((m_pose.block<3, 3>(0, 0) * unbias_acc + last_pose * m_last_unbias_acc) /
       2.0) *
          delta_t;
  // 更新位置
  m_pose.block<3, 1>(0, 3) += (last_vel + m_velocity) / 2.0 * delta_t +
                              0.25 *
                                  (last_pose * m_last_unbias_acc +
                                   m_pose.block<3, 3>(0, 0) * unbias_acc) *
                                  delta_t * delta_t;
  Eigen::Matrix3d tmpr = m_pose.block<3, 3>(0, 0);
  Eigen::Quaterniond tmpq(tmpr);
  std::cout << "Q: " << tmpq.coeffs().transpose() << std::endl;
  std::cout << "posi: " << m_pose.block<3, 1>(0, 3).transpose() << std::endl;
  std::cout << "vel: " << m_velocity.transpose() << std::endl;
  std::cout << "gyr: " << m_last_unbias_gyr.transpose() << std::endl;
  // 开始构建雅各比矩阵
  Eigen::Vector3d cur_acc_NED =
      m_pose.block<3, 3>(0, 0) * (imu_acc - m_accel_bias);
  Eigen::Matrix3d F_23;
  F_23 << 0, -cur_acc_NED[2], cur_acc_NED[1], cur_acc_NED[2], 0,
      -cur_acc_NED[0], -cur_acc_NED[1], cur_acc_NED[0], 0;
  m_F.block<3, 3>(INDEX_STATE_POSI, INDEX_STATE_VEL) =
      Eigen::Matrix3d::Identity();
  m_F.block<3, 3>(INDEX_STATE_VEL, INDEX_STATE_ORI) = F_23;
  m_F.block<3, 3>(INDEX_STATE_ORI, INDEX_STATE_ORI) = Eigen::Matrix3d::Zero();
  m_F.block<3, 3>(INDEX_STATE_VEL, INDEX_STATE_ACC_BIAS) =
      m_pose.block<3, 3>(0, 0);
  m_F.block<3, 3>(INDEX_STATE_ORI, INDEX_STATE_GYRO_BIAS) =
      -m_pose.block<3, 3>(0, 0);
  // 控制矩阵置0
  m_B.block<3, 3>(INDEX_STATE_VEL, 3) = m_pose.block<3, 3>(0, 0);
  m_B.block<3, 3>(INDEX_STATE_ORI, 0) = -m_pose.block<3, 3>(0, 0);
  // 计算状态转移矩阵
  Eigen::Matrix<double, DIM_STATE, DIM_STATE> Fk =
      Eigen::Matrix<double, DIM_STATE, DIM_STATE>::Identity() + m_F * delta_t;
  Eigen::Matrix<double, DIM_STATE, DIM_STATE_NOISE> Bk = m_B * delta_t;
  // 更新状态
  m_X = Fk * m_X;
  // 更新协方差矩阵
  m_P = Fk * m_P * Fk.transpose() + m_B * m_Q * m_B.transpose();

  // 更新imu数据
  m_last_imu_tc = tc;
  m_last_unbias_gyr = unbias_gyr;
  m_last_unbias_acc = unbias_acc;

  pos = m_pose.block<3, 1>(0, 3);
  vel = m_velocity;
  angle_vel = m_last_unbias_gyr;
  q = tmpq;

  return true;
}

void ErrorStateKalmanFilter::correct(Eigen::Vector3d slam_pos,
                                     Eigen::Quaterniond slam_q) {
  std::lock_guard<std::mutex> lock(m_mtx);
  // 构造观测残差
  m_Y.block(0, 0, 3, 1) = slam_pos - m_pose.block<3, 1>(0, 3);
  Eigen::Matrix3d measure_q = slam_q.matrix();
  Eigen::Matrix3d err_q = measure_q.inverse() * m_pose.block<3, 3>(0, 0);
  Eigen::AngleAxisd rotation_vector(err_q);
  m_Y.block(3, 0, 3, 1) = rotation_vector.axis() * rotation_vector.angle();
  // 更新卡尔曼增益
  m_K = m_P * m_G.transpose() *
        (m_G * m_P * m_G.transpose() + m_C * m_R * m_C.transpose()).inverse();
  // 更新协方差
  m_P = (Eigen::Matrix<double, DIM_STATE, DIM_STATE>::Identity() - m_K * m_G) *
        m_P;
  // 更新状态
  m_X = m_X + m_K * (m_Y - m_G * m_X);
  // 修正
  m_pose.block<3, 1>(0, 3) =
      m_pose.block<3, 1>(0, 3) + m_X.block<3, 1>(INDEX_STATE_POSI, 0);
  m_velocity = (m_velocity + m_X.block<3, 1>(INDEX_STATE_VEL, 0));
  // Eigen::Matrix3d phi_hat;
  // phi_hat << 0, - m_X(8, 0), m_X(7, 0),
  //         m_X(8, 0), 0,  -m_X(6, 0),
  //         -m_X(7, 0), m_X(6, 0), 0;
  Eigen::AngleAxisd tmp22;
  tmp22.angle() = m_X.block<3, 1>(6, 0).norm();
  tmp22.axis() = m_X.block<3, 1>(6, 0).normalized();
  Eigen::Matrix3d err_r(tmp22);
  m_pose.block<3, 3>(0, 0) = m_pose.block<3, 3>(0, 0) * err_r.inverse();
  Eigen::Matrix3d tmpr2 = m_pose.block<3, 3>(0, 0);
  Eigen::Quaterniond tmpq2(tmpr2);
  m_gyro_bias += m_X.block<3, 1>(INDEX_STATE_GYRO_BIAS, 0);
  m_accel_bias += m_X.block<3, 1>(INDEX_STATE_ACC_BIAS, 0);
  // 清空误差状态
  m_X = Eigen::Matrix<double, DIM_STATE, 1>::Zero();
}