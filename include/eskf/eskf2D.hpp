#pragma once
#include <Eigen/Dense>
#include <mutex>
#include <string>

class ErrorStateKalmanFilter2D {
 public:
  bool isInitialized = false;
  /**
   * @brief Construct a new Error State Kalman Filter 2D object
   *
   * @param gravity 重力加速度
   * @param pos_noise 位置噪声
   * @param vel_noise 速度噪声
   * @param ori_noise 姿态噪声 (yaw角)
   * @param gyr_bias_noise 陀螺仪偏置噪声 (z轴)
   * @param acc_bias_noise 加速度计偏置噪声
   * @param pos_vel_corr 速度位置协方差
   * @param pos_std slam位置测量标准差
   * @param ori_std slam姿态测量标准差
   * @param gyr_noise 陀螺仪过程噪声
   * @param acc_noise 加速度计过程噪声
   */
  ErrorStateKalmanFilter2D(double gravity, double pos_noise, double vel_noise,
                           double ori_noise, double gyr_bias_noise,
                           double acc_bias_noise, double pos_vel_corr,
                           double pos_std, double ori_std, double gyr_noise,
                           double acc_noise);

  //   /**
  //    * @brief Construct a new Error State Kalman Filter 2D object
  //    *
  //    * @param gravity 重力加速度
  //    * @param pos_std 位置噪声
  //    * @param vel_noise 速度噪声
  //    * @param ori_std 姿态噪声
  //    * @param gyr_noise 陀螺仪偏置噪声
  //    * @param acc_noise 加速度计偏置噪声
  //    * @param filename 文件路径 读取文件中的P gyro_bias accel_bias
  //    */
  //   ErrorStateKalmanFilter2D(double gravity, double pos_std, double ori_std,
  //                            double gyr_noise, double acc_noise,
  //                            const std::string& filename);

  //   /**
  //    * @brief Construct a new Error State Kalman Filter 2D object
  //    *
  //    * @param gravity 重力加速度
  //    * @param pos_std 位置噪声
  //    * @param ori_std 姿态噪声
  //    * @param gyr_noise 陀螺仪偏置噪声
  //    * @param acc_noise 加速度计偏置噪声
  //    * @param P 协方差矩阵 (8x8)
  //    * @param gyro_bias 陀螺仪偏置 (标量，只有z轴)
  //    * @param accel_bias 加速度计偏置 (2D向量，x,y轴)
  //    */
  //   ErrorStateKalmanFilter2D(
  //       double gravity, double pos_std, double ori_std, double gyr_noise,
  //       double acc_noise, const Eigen::Matrix<double, 8, 8> P,
  //       double gyro_bias = 0.0,
  //       Eigen::Vector2d accel_bias = Eigen::Vector2d::Zero());

  //   /**
  //    * @brief 读取yaml文件中的协方差矩阵 gyro_bias accel_bias
  //    *
  //    * @param filename yaml文件路径
  //    * @return true
  //    * @return false
  //    */
  //   bool ReadParameters(const std::string& filename);

  /**
   * @brief 初始化ESKF 2D
   *
   * @param initPose 初始位置 (x, y, yaw)
   * @param initVel 初始速度 (vx, vy)
   * @param tc 初始化时间戳
   */
  void Init(Eigen::Vector3d initPose, Eigen::Vector2d initVel, long long tc);

  /**
   * @brief 预测
   *
   * @param imu_acc imu 加速度数据 (x, y轴)
   * @param imu_gyr imu 陀螺仪数据 (z轴角速度)
   * @param pos 输出位置 (x, y)
   * @param vel 输出速度 (vx, vy)
   * @param angle_vel 输出角速度 (wz)
   * @param yaw 输出航向角
   * @param tc 时间戳
   * @return true 计算成功
   * @return false 两帧之间时间相隔太近或太远，不再计算
   */
  bool Predict(Eigen::Vector2d imu_acc, double imu_gyr, Eigen::Vector2d& pos,
               Eigen::Vector2d& vel, double& angle_vel, double& yaw,
               long long tc);

  /**
   * @brief 纠正
   *
   * @param slam_pos slam获取的位置 (x, y)
   * @param slam_yaw slam获取的航向角
   */
  void correct(Eigen::Vector2d slam_pos, double slam_yaw);

  /**
   * @brief 打印参数
   *
   * @param filename 文件名
   * @return true 打印成功
   * @return false 打印失败
   */
  bool printParameters(const std::string& filename);

 private:
  static const unsigned int DIM_STATE =
      8;  // 状态量数量: [x, y, vx, vy, yaw, gyro_bias_z, acc_bias_x,
          // acc_bias_y]
  static const unsigned int DIM_STATE_NOISE =
      3;  // [gyro_noise, acc_noise_x, acc_noise_y]
  static const unsigned int DIM_MEASUREMENT = 3;  // [x, y, yaw]
  static const unsigned int DIM_MEASUREMENT_NOISE = 3;
  static const unsigned int INDEX_STATE_POSI_X = 0;
  static const unsigned int INDEX_STATE_POSI_Y = 1;
  static const unsigned int INDEX_STATE_VEL_X = 2;
  static const unsigned int INDEX_STATE_VEL_Y = 3;
  static const unsigned int INDEX_STATE_YAW = 4;
  static const unsigned int INDEX_STATE_GYRO_BIAS = 5;
  static const unsigned int INDEX_STATE_ACC_BIAS_X = 6;
  static const unsigned int INDEX_STATE_ACC_BIAS_Y = 7;
  static const unsigned int INDEX_MEASUREMENT_POSI = 0;
  static const unsigned int INDEX_MEASUREMENT_POSI_Y = 1;
  static const unsigned int INDEX_MEASUREMENT_YAW = 2;

  Eigen::Matrix<double, DIM_STATE, 1> m_X =
      Eigen::Matrix<double, DIM_STATE,
                    1>::Zero();  // 状态向量：位置(x,y) 速度(vx,vy) 姿态(yaw)
                                 // 角速度偏置 加速度偏置(x,y)
  Eigen::Matrix<double, DIM_MEASUREMENT, 1> m_Y =
      Eigen::Matrix<double, DIM_MEASUREMENT, 1>::Zero();  // 测量向量
  Eigen::Matrix<double, DIM_STATE, DIM_STATE> m_F =
      Eigen::Matrix<double, DIM_STATE, DIM_STATE>::Zero();  // 状态转移矩阵
  Eigen::Matrix<double, DIM_STATE, DIM_STATE_NOISE> m_B =
      Eigen::Matrix<double, DIM_STATE,
                    DIM_STATE_NOISE>::Zero();  // 噪声输入矩阵
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

  // 定义 IMU 数据 (2D)
  Eigen::Vector2d m_velocity = Eigen::Vector2d::Zero();  // 速度 (vx, vy)
  double m_gyro = 0.0;                                   // 陀螺仪数据 (wz)
  Eigen::Vector2d m_position = Eigen::Vector2d::Zero();  // 位置 (x, y)
  double m_yaw = 0.0;                                    // 航向角
  double m_gyro_bias = 0.0;  // 陀螺仪偏置 (z轴)
  Eigen::Vector2d m_accel_bias =
      Eigen::Vector2d::Zero();  // 加速度计偏置 (x, y轴)
  double m_gravity = 9.81;      // 重力加速度

  // SLAM输入数据
  Eigen::Vector2d m_slam_position =
      Eigen::Vector2d::Zero();  // 激光雷达位置数据
  double m_slam_yaw = 0.0;      // 激光雷达航向角数据

  Eigen::Vector2d m_last_unbias_acc;
  double m_last_unbias_gyr;
  long long m_last_imu_tc;
  std::mutex m_mtx;
};