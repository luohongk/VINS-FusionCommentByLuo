/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <thread>
#include <mutex>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <ceres/ceres.h>
#include <unordered_map>
#include <queue>
#include <opencv2/core/eigen.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "parameters.h"
#include "feature_manager.h"
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "../initial/solve_5pts.h"
#include "../initial/initial_sfm.h"
#include "../initial/initial_alignment.h"
#include "../initial/initial_ex_rotation.h"
#include "../factor/imu_factor.h"
#include "../factor/pose_local_parameterization.h"
#include "../factor/marginalization_factor.h"
#include "../factor/projectionTwoFrameOneCamFactor.h"
#include "../factor/projectionTwoFrameTwoCamFactor.h"
#include "../factor/projectionOneFrameTwoCamFactor.h"
#include "../featureTracker/feature_tracker.h"

class Estimator
{
public:
    Estimator();
    ~Estimator();
    void setParameter();

    // interface
    // 初始哈化第一帧的位姿
    void initFirstPose(Eigen::Vector3d p, Eigen::Matrix3d r);

    // 回调函数中将IMU数据输入到estimator对象中
    void inputIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity);

    // 特征的回调函数，把特征传入到estimate对象中
    void inputFeature(double t, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &featureFrame);

    // 重要函数，在这里面进行的是前端特征提取与后端优化
    void inputImage(double t, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());

    // IMU的预积分函数
    void processIMU(double t, double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);

    // 重要函数，预积分结束后，对后端数据进行优化
    void processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const double header);

    // 重要函数，测量线程
    void processMeasurements();

    // 这个地方就是回调函数对是否使用IMU数据与单目双目数据的切换
    void changeSensorType(int use_imu, int use_stereo);

    // internal
    // 清空状态，把estimator对象中的所有值都设置为默认值
    void clearState();

    // 后端优化前的初始化工作
    bool initialStructure();

    // 视觉初始对准
    // 视觉初始对准的目的是校准相机和IMU之间的初始偏移和尺度
    bool visualInitialAlign();

    // 这个函数的目标是在窗口内寻找与最新帧具有足够匹配点和视差的先前帧
    bool relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l);

    // 这段代码实现了一个滑动窗口的功能，根据一些条件对一些数组进行了移动、交换和删除操作。
    void slideWindow();

    // 主要功能是滑动窗口新帧处理
    void slideWindowNew();

    // 主要功能是根据solver_flag的值执行滑动窗口的旧帧处理
    void slideWindowOld();

    // 位置姿优化函数
    void optimization();

    // 主要功能是将一些类成员变量的值转换为双精度浮点数，并将它们存储在不同的数组中，以便后续使用。
    void vector2double();

    // 主要功能是根据输入的双精度浮点数数组，将值转换为类成员变量的类型，并将其赋值给相应的变量。
    void double2vector();

    bool failureDetection();

    // 获得惯性传感器的数据
    bool getIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accVector,
                        vector<pair<double, Eigen::Vector3d>> &gyrVector);

    // 获得在世界坐标系下的位姿
    void getPoseInWorldFrame(Eigen::Matrix4d &T);

    // 获得在世界坐标系下的位姿(重构函数)
    void getPoseInWorldFrame(int index, Eigen::Matrix4d &T);

    // 预测下一帧中的特征点位置
    void predictPtsInNextFrame();

    // 执行异常值剔除的操作，即从特征点中识别和移除可能是异常的点
    void outliersRejection(set<int> &removeIndex);

    // 用于计算特征点在两个相机帧之间的重投影误差。
    double reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici,
                             Matrix3d &Rj, Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj,
                             double depth, Vector3d &uvi, Vector3d &uvj);

    // 主要功能是在多线程环境下更新最新的状态信息
    void updateLatestStates();

    // 该函数通过使用IMU的线性加速度和角速度数据，根据时间间隔和最新的状态信息，进行快速状态预测。
    void fastPredictIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity);

    // 检查IMU数据是否可用
    bool IMUAvailable(double t);

    // 主要功能是初始化初始IMU的姿态（旋转矩阵）
    void initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector);

    enum SolverFlag
    {
        INITIAL,
        NON_LINEAR
    };

    enum MarginalizationFlag
    {
        MARGIN_OLD = 0,
        MARGIN_SECOND_NEW = 1
    };

    // 互斥锁
    std::mutex mProcess;
    std::mutex mBuf;
    std::mutex mPropagate;

    // 惯性传感器的线加速度与角加速度
    queue<pair<double, Eigen::Vector3d>> accBuf;
    queue<pair<double, Eigen::Vector3d>> gyrBuf;

    // 特征缓冲区队列，每次特征提取完成后，特征都会放入这个缓冲区里面
    queue<pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>>> featureBuf;

    // 当前处理帧的时间，前一帧的时间
    double prevTime, curTime;

    // openExEstimation 用来控制是否启用外参数的估计，根据条件来决定是否进行参数估计或固定。
    bool openExEstimation;

    // std::thread是C++标准库中的一个类，用于创建和管理线程。
    // 通过创建std::thread对象，可以在程序中启动新的线程来执行并发的任务。
    // 通过processThread对象，可以控制该线程的生命周期、启动和停止，以及与其他线程进行同步和通信。
    std::thread trackThread;
    std::thread processThread;

    // 前端跟踪对象
    FeatureTracker featureTracker;

    // 求解器标志，到底是线性的还是非线性的？
    SolverFlag solver_flag;

    // 边缘化标志，在滑动窗口进行优化的时候，到底是边缘化哪一个帧
    MarginalizationFlag marginalization_flag;

    // 表示三维空间中的重力向量
    Vector3d g;

    // ric可能表示两个相机与IMU之间的旋转矩阵
    // 有两个元素则表示是左目，右目
    Matrix3d ric[2];

    // ric可能表示两个相机与IMU之间的平移矩阵
    // 有两个元素则表示是左目，右目
    Vector3d tic[2];

    // 待优化的参数，PVQ
    Vector3d Ps[(WINDOW_SIZE + 1)];
    Vector3d Vs[(WINDOW_SIZE + 1)];
    Matrix3d Rs[(WINDOW_SIZE + 1)];

    // 用于存储加速度计的零偏信息。
    Vector3d Bas[(WINDOW_SIZE + 1)];

    // 用于存储陀螺仪的零偏信息。
    Vector3d Bgs[(WINDOW_SIZE + 1)];

    // 待估计的参数，表示时间偏移量的参数。
    double td;
    
    // 用于存储旋转矩阵
    Matrix3d back_R0, last_R, last_R0;

    // 用于存储位置信息，没有搞懂这个back_P0有什么用
    // last_P, last_P0应该是用于更新位置和姿态的临时存储量
    Vector3d back_P0, last_P, last_P0;

    // 这个用于存储滑动窗口的时间戳
    double Headers[(WINDOW_SIZE + 1)];

    // 这个用于存储滑动窗口的预积分量
    IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)];

    // 存储加速度计和陀螺仪的初始值。
    Vector3d acc_0, gyr_0;

    // 滑动窗口内的时间差列表
    vector<double> dt_buf[(WINDOW_SIZE + 1)];

    // 滑动窗口内的线加速度缓冲区列表
    vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];

    // 滑动窗口内的陀螺加速度缓冲区列表
    vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];

    // 帧记数目，比如目前是来的第几张图像那就是几
    int frame_count;


    int sum_of_outlier, sum_of_back, sum_of_front, sum_of_invalid;

    //todo 没有太懂这个inputImageCnt与frame_count有什么区别？
    int inputImageCnt;

    // 地图点管理器
    FeatureManager f_manager;
    MotionEstimator m_estimator;

    // 用于初始外参旋转估计。
    InitialEXRotation initial_ex_rotation;

    // 表示是否是第一个IMU的布尔变量
    bool first_imu;

    // 是否有效，是否是关键帧
    // todo 没看懂这个变量有什么用
    bool is_valid, is_key;
    bool failure_occur;

    // 用于存储点云信息。地图点
    vector<Vector3d> point_cloud;

    // 用于存储边缘化后地图点的信息
    vector<Vector3d> margin_cloud;
    vector<Vector3d> key_poses;

    // 初始的时间戳
    double initial_timestamp;

    double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];
    double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
    double para_Feature[NUM_OF_F][SIZE_FEATURE];
    double para_Ex_Pose[2][SIZE_POSE];
    double para_Retrive_Pose[SIZE_POSE];
    double para_Td[1][1];
    double para_Tr[1][1];

    int loop_window_index;

    MarginalizationInfo *last_marginalization_info;
    vector<double *> last_marginalization_parameter_blocks;

    map<double, ImageFrame> all_image_frame;
    IntegrationBase *tmp_pre_integration;

    // 用于存储初始位置信息。
    Eigen::Vector3d initP;

    // 用于存储初始旋转矩阵信息。
    Eigen::Matrix3d initR;

    // 表示最新时间戳的双精度浮点数。
    double latest_time;

    // 用于存储最新位置、速度、加速度计和陀螺仪信息。
    Eigen::Vector3d latest_P, latest_V, latest_Ba, latest_Bg, latest_acc_0, latest_gyr_0;
    
    // 用于存储最新姿态信息。
    Eigen::Quaterniond latest_Q;
    
    // 表示初始姿态标志的布尔变量。
    bool initFirstPoseFlag;

    // 表示初始线程标志的布尔变量。
    bool initThreadFlag;
};
