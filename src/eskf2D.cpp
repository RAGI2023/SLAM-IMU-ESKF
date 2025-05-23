#include "eskf/eskf2D.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>

ErrorStateKalmanFilter2D::ErrorStateKalmanFilter2D(
    double gravity, double pos_noise, double vel_noise, double ori_noise,
    double gyr_bias_noise, double acc_bias_noise, double pos_vel_corr,
    double pos_std, double ori_std, double gyr_noise, double acc_noise) {
  m_gravity = gravity;

  // 初始化协方差矩阵 - 位置速度相关性
  m_P(INDEX_STATE_POSI_X, INDEX_STATE_VEL_X) = pos_vel_corr;
  m_P(INDEX_STATE_VEL_X, INDEX_STATE_POSI_X) = pos_vel_corr;
  m_P(INDEX_STATE_POSI_Y, INDEX_STATE_VEL_Y) = pos_vel_corr;
  m_P(INDEX_STATE_VEL_Y, INDEX_STATE_POSI_Y) = pos_vel_corr;

  // 对角线方差
  m_P(INDEX_STATE_POSI_X, INDEX_STATE_POSI_X) = pos_noise * pos_noise;
  m_P(INDEX_STATE_POSI_Y, INDEX_STATE_POSI_Y) = pos_noise * pos_noise;
  m_P(INDEX_STATE_VEL_X, INDEX_STATE_VEL_X) = vel_noise * vel_noise;
  m_P(INDEX_STATE_VEL_Y, INDEX_STATE_VEL_Y) = vel_noise * vel_noise;
  m_P(INDEX_STATE_YAW, INDEX_STATE_YAW) = ori_noise * ori_noise;
  m_P(INDEX_STATE_GYRO_BIAS, INDEX_STATE_GYRO_BIAS) =
      gyr_bias_noise * gyr_bias_noise;
  m_P(INDEX_STATE_ACC_BIAS_X, INDEX_STATE_ACC_BIAS_X) =
      acc_bias_noise * acc_bias_noise;
  m_P(INDEX_STATE_ACC_BIAS_Y, INDEX_STATE_ACC_BIAS_Y) =
      acc_bias_noise * acc_bias_noise;

  // 初始化测量噪声矩阵
  m_R.block<2, 2>(0, 0) = Eigen::Matrix2d::Identity() * pos_std * pos_std;
  m_R(2, 2) = ori_std * ori_std;

  // 初始化过程噪声协方差矩阵
  m_Q(0, 0) = gyr_noise * gyr_noise;
  m_Q.block<2, 2>(1, 1) = Eigen::Matrix2d::Identity() * acc_noise * acc_noise;
  m_Q(2, 2) = acc_noise * acc_noise;

  // 初始化状态为0
  m_X = Eigen::Matrix<double, DIM_STATE, 1>::Zero();

  // 状态转移矩阵初始化为0
  m_F = Eigen::Matrix<double, DIM_STATE, DIM_STATE>::Zero();

  // 测量噪声的变换矩阵
  m_C = Eigen::Matrix<double, DIM_MEASUREMENT_NOISE,
                      DIM_MEASUREMENT_NOISE>::Identity();

  // 测量矩阵
  m_G = Eigen::Matrix<double, DIM_MEASUREMENT, DIM_STATE>::Zero();
  m_G.block<2, 2>(INDEX_MEASUREMENT_POSI, INDEX_MEASUREMENT_POSI) =
      Eigen::Matrix2d::Identity();
  m_G(INDEX_MEASUREMENT_YAW, INDEX_STATE_YAW) = 1.0;
}

void ErrorStateKalmanFilter2D::Init(Eigen::Vector3d initPose,
                                    Eigen::Vector2d initVel, long long tc) {
  m_position = initPose.head(2);  // x, y
  m_yaw = initPose(2);            // yaw
  m_velocity = initVel;           // vx, vy

  // 记录时间戳
  m_last_imu_tc = tc;

  // 偏置初始化
  m_last_unbias_acc = Eigen::Vector2d::Zero();
  m_last_unbias_gyr = 0.0;

  isInitialized = true;
  std::cout << "Init pose: [" << m_position.transpose() << ", " << m_yaw << "]"
            << std::endl;
  std::cout << "Init velocity: [" << m_velocity.transpose() << "]" << std::endl;
}

bool ErrorStateKalmanFilter2D::Predict(Eigen::Vector2d imu_acc, double imu_gyr,
                                       Eigen::Vector2d& pos,
                                       Eigen::Vector2d& vel, double& angle_vel,
                                       double& yaw, long long tc) {
  // 计算时间差
  double delta_t = (tc - m_last_imu_tc) / 1000000000.0;
  if (delta_t > 0.02) {
    m_last_imu_tc = tc;
    std::cout << "IMU delta t " << delta_t << std::endl;
    return false;
  }

  std::lock_guard<std::mutex> lock(m_mtx);

  std::cout << "delta sec: " << delta_t << std::endl;

  // 陀螺仪数据去除偏置
  double unbias_gyr = imu_gyr - m_gyro_bias;

  // 计算角度增量 (2D只有yaw轴)
  double delta_yaw = (unbias_gyr + m_last_unbias_gyr) / 2.0 * delta_t;

  // 更新yaw角
  double last_yaw = m_yaw;
  m_yaw += delta_yaw;

  // 归一化yaw角到[-pi, pi]
  while (m_yaw > M_PI) m_yaw -= 2 * M_PI;
  while (m_yaw < -M_PI) m_yaw += 2 * M_PI;

  // 去除加速度偏置 (2D，不包含重力，因为重力在z轴)
  Eigen::Vector2d unbias_acc = imu_acc - m_accel_bias;

  // 计算旋转矩阵 (2D)
  double cos_yaw = cos(m_yaw);
  double sin_yaw = sin(m_yaw);
  double cos_last_yaw = cos(last_yaw);
  double sin_last_yaw = sin(last_yaw);

  Eigen::Matrix2d R_current, R_last;
  R_current << cos_yaw, -sin_yaw, sin_yaw, cos_yaw;
  R_last << cos_last_yaw, -sin_last_yaw, sin_last_yaw, cos_last_yaw;

  // 更新速度 (将加速度转换到世界坐标系)
  Eigen::Vector2d last_vel = m_velocity;
  m_velocity =
      last_vel +
      ((R_current * unbias_acc + R_last * m_last_unbias_acc) / 2.0) * delta_t;

  // 更新位置
  m_position += (last_vel + m_velocity) / 2.0 * delta_t +
                0.25 * (R_last * m_last_unbias_acc + R_current * unbias_acc) *
                    delta_t * delta_t;

  std::cout << "yaw: " << m_yaw << std::endl;
  std::cout << "position: " << m_position.transpose() << std::endl;
  std::cout << "velocity: " << m_velocity.transpose() << std::endl;
  std::cout << "gyro: " << m_last_unbias_gyr << std::endl;

  // 构建状态转移矩阵的雅各比
  // F矩阵 (8x8)
  m_F = Eigen::Matrix<double, DIM_STATE, DIM_STATE>::Zero();

  // 位置对速度的导数
  m_F(INDEX_STATE_POSI_X, INDEX_STATE_VEL_X) = 1.0;
  m_F(INDEX_STATE_POSI_Y, INDEX_STATE_VEL_Y) = 1.0;

  // 速度对姿态的导数 (考虑加速度在body frame到world frame的转换)
  Eigen::Vector2d acc_world = R_current * (imu_acc - m_accel_bias);
  m_F(INDEX_STATE_VEL_X, INDEX_STATE_YAW) = -acc_world(1);  // -ay
  m_F(INDEX_STATE_VEL_Y, INDEX_STATE_YAW) = acc_world(0);   // ax

  // 速度对加速度偏置的导数
  m_F(INDEX_STATE_VEL_X, INDEX_STATE_ACC_BIAS_X) = -cos_yaw;
  m_F(INDEX_STATE_VEL_X, INDEX_STATE_ACC_BIAS_Y) = sin_yaw;
  m_F(INDEX_STATE_VEL_Y, INDEX_STATE_ACC_BIAS_X) = -sin_yaw;
  m_F(INDEX_STATE_VEL_Y, INDEX_STATE_ACC_BIAS_Y) = -cos_yaw;

  // 姿态对陀螺仪偏置的导数
  m_F(INDEX_STATE_YAW, INDEX_STATE_GYRO_BIAS) = -1.0;

  // 噪声输入矩阵 B (8x4)
  m_B = Eigen::Matrix<double, DIM_STATE, DIM_STATE_NOISE>::Zero();
  m_B(INDEX_STATE_VEL_X, 1) = cos_yaw;   // acc noise x
  m_B(INDEX_STATE_VEL_X, 2) = -sin_yaw;  // acc noise y
  m_B(INDEX_STATE_VEL_Y, 1) = sin_yaw;   // acc noise x
  m_B(INDEX_STATE_VEL_Y, 2) = cos_yaw;   // acc noise y
  m_B(INDEX_STATE_YAW, 0) = 1.0;         // gyro noise

  // m_B(INDEX_STATE_VEL_X, 1) *= 2;  // acc noise x
  // m_B(INDEX_STATE_VEL_X, 2) *= 2;  // acc noise y
  // m_B(INDEX_STATE_VEL_Y, 1) *= 2;  // acc noise x
  // m_B(INDEX_STATE_VEL_Y, 2) *= 2;  // acc noise y

  // 计算离散化的状态转移矩阵
  Eigen::Matrix<double, DIM_STATE, DIM_STATE> Fk =
      Eigen::Matrix<double, DIM_STATE, DIM_STATE>::Identity() + m_F * delta_t;
  Eigen::Matrix<double, DIM_STATE, DIM_STATE_NOISE> Bk = m_B * delta_t;

  // 更新误差状态
  m_X = Fk * m_X;

  // 更新协方差矩阵
  m_P = Fk * m_P * Fk.transpose() + Bk * m_Q * Bk.transpose();

  // 更新IMU数据记录
  m_last_imu_tc = tc;
  m_last_unbias_gyr = unbias_gyr;
  m_last_unbias_acc = unbias_acc;

  // 输出结果
  pos = m_position;
  vel = m_velocity;
  angle_vel = m_last_unbias_gyr;
  yaw = m_yaw;

  return true;
}

void ErrorStateKalmanFilter2D::correct(Eigen::Vector2d slam_pos,
                                       double slam_yaw) {
  std::lock_guard<std::mutex> lock(m_mtx);

  // 构造观测残差
  m_Y(INDEX_MEASUREMENT_POSI, 0) = slam_pos(0) - m_position(0);
  m_Y(INDEX_MEASUREMENT_POSI_Y, 0) = slam_pos(1) - m_position(1);

  // 角度残差处理 (归一化到[-pi, pi])
  double yaw_residual = slam_yaw - m_yaw;
  while (yaw_residual > M_PI) yaw_residual -= 2 * M_PI;
  while (yaw_residual < -M_PI) yaw_residual += 2 * M_PI;
  m_Y(INDEX_MEASUREMENT_YAW, 0) = yaw_residual;

  // 更新卡尔曼增益
  Eigen::Matrix<double, DIM_MEASUREMENT, DIM_MEASUREMENT> S =
      m_G * m_P * m_G.transpose() + m_C * m_R * m_C.transpose();
  m_K = m_P * m_G.transpose() * S.inverse();

  // 更新协方差
  m_P = (Eigen::Matrix<double, DIM_STATE, DIM_STATE>::Identity() - m_K * m_G) *
        m_P;

  // 更新误差状态
  m_X = m_X + m_K * (m_Y - m_G * m_X);

  // 修正状态
  m_position(0) += m_X(INDEX_STATE_POSI_X, 0);
  m_position(1) += m_X(INDEX_STATE_POSI_Y, 0);
  m_velocity(0) += m_X(INDEX_STATE_VEL_X, 0);
  m_velocity(1) += m_X(INDEX_STATE_VEL_Y, 0);
  m_yaw += m_X(INDEX_STATE_YAW, 0);

  // 归一化yaw角
  while (m_yaw > M_PI) m_yaw -= 2 * M_PI;
  while (m_yaw < -M_PI) m_yaw += 2 * M_PI;

  // 修正偏置
  m_gyro_bias += m_X(INDEX_STATE_GYRO_BIAS, 0);
  m_accel_bias(0) += m_X(INDEX_STATE_ACC_BIAS_X, 0);
  m_accel_bias(1) += m_X(INDEX_STATE_ACC_BIAS_Y, 0);

  // 清空误差状态
  m_X = Eigen::Matrix<double, DIM_STATE, 1>::Zero();

  std::cout << "After correction - Position: " << m_position.transpose()
            << ", Yaw: " << m_yaw << std::endl;
}

bool ErrorStateKalmanFilter2D::printParameters(const std::string& filename) {
  std::ofstream ofs(filename);
  if (!ofs.is_open()) {
    std::cerr << "Failed to open file for writing: " << filename << std::endl;
    return false;
  }

  ofs << std::fixed << std::setprecision(8);

  // 打印 gyro_bias (1个数，z轴)
  ofs << "# gyro_bias\n";
  ofs << m_gyro_bias << "\n";

  // 打印 accel_bias (2个数，x和y轴)
  ofs << "# accel_bias\n";
  for (int i = 0; i < 2; ++i) {
    ofs << m_accel_bias(i);
    if (i != 1) ofs << " ";
  }
  ofs << "\n";

  // 打印 P 矩阵 (8行，每行8个数)
  ofs << "# P_matrix\n";
  for (unsigned int i = 0; i < DIM_STATE; ++i) {
    for (unsigned int j = 0; j < DIM_STATE; ++j) {
      ofs << m_P(i, j);
      if (j != DIM_STATE - 1) ofs << " ";
    }
    ofs << "\n";
  }

  ofs.close();
  std::cout << "Successfully saved parameters to " << filename << std::endl;
  return true;
}