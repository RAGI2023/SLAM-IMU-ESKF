#pragma once
// #include <Eigen/Dense>
#include "Eigen/Dense"
#include <mutex>
#include <string>

class ErrorStateKalmanFilter {
 public:
  bool isInitialized = false;
  /**
   * @brief Construct a new Error State Kalman Filter:: Error State Kalman
   * Filter object
   *
   * @param gravity 重力加速度
   * @param pos_noise 位置噪声
   * @param vel_noise 速度噪声
   * @param ori_noise 姿态噪声
   * @param gyr_bias_noise 陀螺仪偏置噪声
   * @param acc_bias_noise 加速度计偏置噪声
   * @param pos_vel_corr 速度位置协方差
   * @param pos_std slam位置测量标准差
   * @param ori_std slam姿态测量标准差
   * @param gyr_noise 陀螺仪过程噪声
   * @param acc_noise 加速度计过程噪声
   */
  ErrorStateKalmanFilter(double gravity, double pos_noise, double vel_noise,
                         double ori_noise, double gyr_bias_noise,
                         double acc_bias_noise, double pos_vel_corr,
                         double pos_std, double ori_std, double gyr_noise,
                         double acc_noise);

  /**
   * @brief Construct a new Error State Kalman Filter object
   *
   * @param gravity 重力加速度
   * @param pos_std 位置噪声
   * @param vel_noise 速度噪声
   * @param ori_std 姿态噪声
   * @param gyr_noise 陀螺仪偏置噪声
   * @param acc_noise 加速度计偏置噪声
   * @param filename 文件路径 读取文件中的P gyro_bias accel_bias
   */
  ErrorStateKalmanFilter(double gravity, double pos_std, double ori_std,
                         double gyr_noise, double acc_noise,
                         const std::string& filename);
  /**
   * @brief Construct a new Error State Kalman Filter object
   *
   * @param gravity 重力加速度
   * @param pos_std 位置噪声
   * @param ori_std 姿态噪声
   * @param gyr_noise 陀螺仪偏置噪声
   * @param acc_noise 加速度计偏置噪声
   * @param P 协方差矩阵
   * @param gyro_bias 陀螺仪偏置
   * @param accel_bias 加速度计偏置
   */
  ErrorStateKalmanFilter(double gravity, double pos_std, double ori_std,
                         double gyr_noise, double acc_noise,
                         const Eigen::Matrix<double, 15, 15> P,
                         Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero(),
                         Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero());
  /**
   * @brief 读取yaml文件中的协方差矩阵 gyro_bias accel_bias
   *
   * @param filename yaml文件路径
   * @return true
   * @return false
   */
  bool ReadParameters(const std::string& filename);

  /**
   * @brief 初始化ESKF
   *
   * @param initPose 初始位置 \begin{matrix} R & t \\ 0 & 1 \end{matrix}
   * @param initVel 初始速度
   * @param tc 初始化时间戳
   */
  void Init(Eigen::Matrix4d initPose, Eigen::Vector3d initVel, long long tc);
  /**
   * @brief 预测
   *
   * @param imu_acc imu 加速度数据
   * @param imu_gyr imu 陀螺仪数据
   * @param pos 输出位置
   * @param vel 输出速度
   * @param angle_vel 输出角速度
   * @param q 输出旋转
   * @param tc 时间戳
   * @return true 计算成功
   * @return false 两帧之间时间相隔太近或太远，不再计算
   */
  bool Predict(Eigen::Vector3d imu_acc, Eigen::Vector3d imu_gyr,
               Eigen::Vector3d& pos, Eigen::Vector3d& vel,
               Eigen::Vector3d& angle_vel, Eigen::Quaterniond& q, long long tc);
  /**
   * @brief 纠正
   *
   * @param slam_pos slam获取的位置
   * @param slam_q slam获取的旋转
   */
  void correct(Eigen::Vector3d slam_pos, Eigen::Quaterniond slam_q);

  /**
   * @brief 打印参数
   *
   * @param filename 文件名
   * @return true 打印成功
   * @return false 打印失败
   */
  bool printParameters(const std::string& filename);

 private:
  static const unsigned int DIM_STATE = 15;  // 状态量数量
  static const unsigned int DIM_STATE_NOISE = 6;
  static const unsigned int DIM_MEASUREMENT = 6;
  static const unsigned int DIM_MEASUREMENT_NOISE = 6;
  static const unsigned int INDEX_STATE_POSI = 0;
  static const unsigned int INDEX_STATE_VEL = 3;
  static const unsigned int INDEX_STATE_ORI = 6;
  static const unsigned int INDEX_STATE_GYRO_BIAS = 9;
  static const unsigned int INDEX_STATE_ACC_BIAS = 12;
  static const unsigned int INDEX_MEASUREMENT_POSI = 0;

  Eigen::Matrix<double, DIM_STATE, 1> m_X =
      Eigen::Matrix<double, DIM_STATE,
                    1>::Zero();  // 真值：位置 速度 姿态 角速度偏置 加速度偏置
  Eigen::Matrix<double, DIM_MEASUREMENT, 1> m_Y =
      Eigen::Matrix<double, DIM_MEASUREMENT, 1>::Zero();  // 测量矩阵
  Eigen::Matrix<double, DIM_STATE, DIM_STATE> m_F =
      Eigen::Matrix<double, DIM_STATE, DIM_STATE>::Zero();  // 状态转移矩阵
  Eigen::Matrix<double, DIM_STATE, DIM_STATE_NOISE> m_B =
      Eigen::Matrix<double, DIM_STATE, DIM_STATE_NOISE>::
          Zero();  // 噪声输入矩阵，将噪声输入到系统的影响
  Eigen::Matrix<double, DIM_STATE_NOISE, DIM_STATE_NOISE> m_Q =
      Eigen::Matrix<double, DIM_STATE_NOISE,
                    DIM_STATE_NOISE>::Zero();  // 过程噪声协方差矩阵
  Eigen::Matrix<double, DIM_STATE, DIM_STATE> m_P =
      Eigen::Matrix<double, DIM_STATE, DIM_STATE>::Zero();  // 状态协方差矩阵
  Eigen::Matrix<double, DIM_STATE, DIM_MEASUREMENT> m_K =
      Eigen::Matrix<double, DIM_STATE, DIM_MEASUREMENT>::Zero();  // 卡尔曼增益
  Eigen::Matrix<double, DIM_MEASUREMENT, DIM_STATE> m_G =
      Eigen::Matrix<double, DIM_MEASUREMENT, DIM_STATE>::Zero();  // 测量矩阵
  Eigen::Matrix<double, DIM_MEASUREMENT, DIM_MEASUREMENT> m_R =
      Eigen::Matrix<double, DIM_MEASUREMENT,
                    DIM_MEASUREMENT>::Zero();  // 测量噪声
  Eigen::Matrix<double, DIM_MEASUREMENT_NOISE, DIM_MEASUREMENT_NOISE> m_C =
      Eigen::Matrix<double, DIM_MEASUREMENT_NOISE,
                    DIM_MEASUREMENT_NOISE>::Zero();  // 测量噪声的变换矩阵

  // 定义 IMU 数据
  Eigen::Vector3d m_velocity = Eigen::Vector3d::Zero();  // 速度
  Eigen::Vector3d m_gyro = Eigen::Vector3d::Zero();      // 陀螺仪数据
  Eigen::Matrix4d m_pose =
      Eigen::Matrix4d::Identity();  // 姿态矩阵 (位置 + 旋转)
  Eigen::Vector3d m_gyro_bias = Eigen::Vector3d::Zero();   // 陀螺仪偏置
  Eigen::Vector3d m_accel_bias = Eigen::Vector3d::Zero();  // 加速度计偏置
  Eigen::Vector3d m_g = Eigen::Vector3d::Zero();           // 重力加速度

  // SLAM输入数据
  Eigen::Vector3d m_slam_position = Eigen::Vector3d::Zero();  // 雷达位置数据

  Eigen::Vector3d m_last_unbias_acc;
  Eigen::Vector3d m_last_unbias_gyr;
  long long m_last_imu_tc;
  std::mutex m_mtx;
};