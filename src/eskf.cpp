#include "eskf/eskf.hpp"
#include <iostream>

/**
 * @brief Construct a new Error State Kalman Filter:: Error State Kalman Filter object
 * 
 * @param gravity 重力加速度
 * @param pos_noise 位置噪声
 * @param vel_noise 速度噪声
 * @param ori_noise 姿态噪声
 * @param gyr_bias_noise 陀螺仪偏置噪声
 * @param acc_bias_noise 加速度计偏置噪声
 * @param pos_std 位置测量标准差
 * @param ori_std 姿态测量标准差
 * @param gyr_noise 陀螺仪过程噪声
 * @param acc_noise 加速度计过程噪声
 */
ErrorStateKalmanFilter::ErrorStateKalmanFilter(double gravity, double pos_noise, double vel_noise,
    double ori_noise, double gyr_bias_noise,
    double acc_bias_noise, double pos_std, double ori_std,
    double gyr_noise, double acc_noise)
    {
        m_g = Eigen::Vector3d(0.0, 0.0, gravity);
        // 初始化协方差矩阵，协方差设为0，设置方差
        m_P.block<3, 3>(INDEX_STATE_POSI, INDEX_STATE_POSI) = Eigen::Matrix3d::Identity() * pos_noise * pos_noise;
        m_P.block<3, 3>(INDEX_STATE_VEL, INDEX_STATE_VEL) = Eigen::Matrix3d::Identity() * vel_noise * vel_noise;
        m_P.block<3, 3>(INDEX_STATE_ORI, INDEX_STATE_ORI) = Eigen::Matrix3d::Identity() * ori_noise * ori_noise;
        m_P.block<3, 3>(INDEX_STATE_GYRO_BIAS, INDEX_STATE_GYRO_BIAS) = Eigen::Matrix3d::Identity() * gyr_bias_noise * gyr_bias_noise;
        m_P.block<3, 3>(INDEX_STATE_ACC_BIAS, INDEX_STATE_ACC_BIAS) = Eigen::Matrix3d::Identity() * acc_bias_noise * acc_bias_noise;
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
        m_C = Eigen::Matrix<double, DIM_MEASUREMENT_NOISE, DIM_MEASUREMENT_NOISE>::Identity();
        // 从状态中提取位置和姿态信息，将X投影到测量空间Y
        m_G.block<3, 3>(INDEX_MEASUREMENT_POSI, INDEX_MEASUREMENT_POSI) = Eigen::Matrix3d::Identity();
        m_G.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity();
    }

    /**
     * @brief 初始化ESKF
     * 
     * @param initPose 初始位置
     * @param initVel 初始速度
     * @param tc 初始化时间戳
     */
    void ErrorStateKalmanFilter::Init(Eigen::Matrix4d initPose, Eigen::Vector3d initVel, long long tc)
    {
        /*
        |R t|
        |0 1|
        */
        m_pose = initPose;
        m_velocity = initVel;
        // 记录时间戳
        m_last_imu_tc = tc;
        // 偏置置0
        m_last_unbias_acc = Eigen::Vector3d::Zero();
        m_last_unbias_gyr = Eigen::Vector3d::Zero();
        std::cout<<"init pose: \n"<<m_pose<<std::endl;
    }
