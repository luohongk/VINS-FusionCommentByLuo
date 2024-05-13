/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "estimator.h"
#include "../utility/visualization.h"

Estimator::Estimator() : f_manager{Rs}
{
    ROS_INFO("init begins");
    initThreadFlag = false;
    clearState();
}

Estimator::~Estimator()
{
    if (MULTIPLE_THREAD)
    {
        processThread.join();
        printf("join thread \n");
    }
}

// 清理状态量
void Estimator::clearState()
{
    // 一帧处理完毕，把加速度，角速度，特征帧出队
    mProcess.lock();
    while (!accBuf.empty())
        accBuf.pop();
    while (!gyrBuf.empty())
        gyrBuf.pop();
    while (!featureBuf.empty())
        featureBuf.pop();

    prevTime = -1;
    curTime = 0;
    openExEstimation = 0;
    initP = Eigen::Vector3d(0, 0, 0);
    initR = Eigen::Matrix3d::Identity();
    inputImageCnt = 0;
    initFirstPoseFlag = false;

    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();

        if (pre_integrations[i] != nullptr)
        {
            delete pre_integrations[i];
        }
        pre_integrations[i] = nullptr;
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d::Zero();
        ric[i] = Matrix3d::Identity();
    }

    first_imu = false,
    sum_of_back = 0;
    sum_of_front = 0;
    frame_count = 0;
    solver_flag = INITIAL;
    initial_timestamp = 0;
    all_image_frame.clear();

    if (tmp_pre_integration != nullptr)
        delete tmp_pre_integration;
    if (last_marginalization_info != nullptr)
        delete last_marginalization_info;

    tmp_pre_integration = nullptr;
    last_marginalization_info = nullptr;
    last_marginalization_parameter_blocks.clear();

    f_manager.clearState();

    failure_occur = 0;

    mProcess.unlock();
}

// 设置Estimator类的参数
void Estimator::setParameter()
{
    mProcess.lock(); // 上锁以确保多线程安全访问
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = TIC[i]; // 设置相机i的平移向量
        ric[i] = RIC[i]; // 设置相机i的旋转矩阵
        cout << " exitrinsic cam " << i << endl
             << ric[i] << endl
             << tic[i].transpose() << endl; // 输出相机i的外参信息
    }
    f_manager.setRic(ric); // 设置特征管理器的ric参数
    // 设置三种因子函数的信息矩阵
    ProjectionTwoFrameOneCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionTwoFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionOneFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    td = TD;                                          // 设置时间偏置参数
    g = G;                                            // 设置重力加速度向量
    cout << "set g " << g.transpose() << endl;        // 输出设置后的重力加速度向量
    featureTracker.readIntrinsicParameter(CAM_NAMES); // 读取相机的内参参数

    std::cout << "MULTIPLE_THREAD is " << MULTIPLE_THREAD << '\n'; // 输出是否启用多线程的信息
    if (MULTIPLE_THREAD && !initThreadFlag)                        // 如果启用了多线程且未初始化线程标志为假
    {
        initThreadFlag = true;                                              // 设置初始化线程标志为真
        processThread = std::thread(&Estimator::processMeasurements, this); // 初始化处理测量数据的线程
    }
    mProcess.unlock(); // 解锁以释放多线程安全访问
}

void Estimator::changeSensorType(int use_imu, int use_stereo)
{
    bool restart = false;
    mProcess.lock();
    if (!use_imu && !use_stereo)
        printf("at least use two sensors! \n");
    else
    {
        if (USE_IMU != use_imu)
        {
            USE_IMU = use_imu;
            if (USE_IMU)
            {
                // reuse imu; restart system
                restart = true;
            }
            else
            {
                if (last_marginalization_info != nullptr)
                    delete last_marginalization_info;

                tmp_pre_integration = nullptr;
                last_marginalization_info = nullptr;
                last_marginalization_parameter_blocks.clear();
            }
        }

        STEREO = use_stereo;
        printf("use imu %d use stereo %d\n", USE_IMU, STEREO);
    }
    mProcess.unlock();
    if (restart)
    {
        clearState();
        setParameter();
    }
}

// Estimator 类的 inputImage 方法
void Estimator::inputImage(double t, const cv::Mat &_img, const cv::Mat &_img1)
{
    inputImageCnt++; // 增加图片输入次数计数

    // 保存特征帧的字典 featureFrame 和特征追踪计时器
    // map<特征点ID， vector<相机ID， 特征点信息>>
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;

    // TicToc为一个计时器类，用于统计程序运行时间
    TicToc featureTrackerTime;

    // 如果 _img1 为空，则调用 featureTracker 的 trackImage 方法对单张图像 _img 进行特征追踪
    // 否则调用 trackImage 方法对图像 _img 和 _img1 进行特征追踪
    if (_img1.empty())

        //  前后帧匹配的特征点具有相同的特征点ID；
        // 左右目匹配的特征点也具有相同的特征点ID，存储在featureFrame中的不同相机Id下(featureFrame[特征点Id].size() > 1)
        featureFrame = featureTracker.trackImage(t, _img);
    else
        featureFrame = featureTracker.trackImage(t, _img, _img1);

    // 如果 SHOW_TRACK 为真，则获取特征追踪图像并发布
    // 在rviz可视化界面中展示图像
    if (SHOW_TRACK)
    {
        cv::Mat imgTrack = featureTracker.getTrackImage();
        pubTrackImage(imgTrack, t); // 发布特征追踪图像
    }

    // 如果 MULTIPLE_THREAD 为真，并且输入次数为偶数，则将特征帧数据存入 featureBuf 中
    // 否则直接将特征帧数据存入 featureBuf，并调用 processMeasurements 方法对测量数据进行处理并打印处理时间
    // MULTIPLE_THREAD是否启用多线程处理，这个是从config文件中读取的

    // mark 此处不是太理解为什么要对输入的图片数做一个偶数判断
    if (MULTIPLE_THREAD)
    {
        //  会检查输入图片的次数，如果是偶数，则将特征帧数据存入 featureBuf 中。这个机制可能是为了优化多线程处理
        if (inputImageCnt % 2 == 0)
        {
            mBuf.lock();                                 // 加锁
            featureBuf.push(make_pair(t, featureFrame)); // 存入特征帧数据
            mBuf.unlock();                               // 解锁
        }
    }
    else
    {
        mBuf.lock();                                 // 加锁
        featureBuf.push(make_pair(t, featureFrame)); // 存入特征帧数据
        mBuf.unlock();                               // 解锁

        TicToc processTime;

        // 这里又是一个新的线程，用于处理测量数据，只要有新的一帧放到图像的缓冲区，就会进行测量线程的处理
        processMeasurements();                           // 处理测量数据
        printf("process time: %f\n", processTime.toc()); // 打印处理时间
    }
}

void Estimator::inputIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity)
{
    mBuf.lock();
    // 把线加速度，角速度存入缓冲区
    accBuf.push(make_pair(t, linearAcceleration));
    gyrBuf.push(make_pair(t, angularVelocity));
    // printf("input imu with time %f \n", t);
    mBuf.unlock();

    // 如果是非线性问题的求解
    if (solver_flag == NON_LINEAR)
    {
        mPropagate.lock();
        fastPredictIMU(t, linearAcceleration, angularVelocity);
        pubLatestOdometry(latest_P, latest_Q, latest_V, t);
        mPropagate.unlock();
    }
}

// 这个函数把提取出来的特征帧放入特征帧缓冲队列中，如果是从featuretracker这个节点中拿到特征，就直接进行测量线程
// 但是实际情况不是这样的，这个函数仅限于新增特征提取与匹配节点的时候用
void Estimator::inputFeature(double t, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &featureFrame)
{
    mBuf.lock();
    // 这里直接把特征帧数据存入特征帧缓冲队列中
    featureBuf.push(make_pair(t, featureFrame));
    mBuf.unlock();

    if (!MULTIPLE_THREAD)
        processMeasurements();
}

// Estimator类的成员函数，用于获取指定时间范围内的IMU数据
bool Estimator::getIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accVector,
                               vector<pair<double, Eigen::Vector3d>> &gyrVector)
{
    // 先看看imu数据是不是空的，是空的打印一下提示信息，然后返回false。
    // 空的就代表没有拿到imu数据
    if (accBuf.empty())
    {
        printf("not receive imu\n");
        return false;
    }
    // 若IMU数据为空，则输出提示信息并返回false
    // printf("get imu from %f %f\n", t0, t1);
    // printf("imu fornt time %f   imu end time %f\n", accBuf.front().first, accBuf.back().first);
    // 如果 t1 小于等于缓冲区accBuf中最后一条IMU数据的时间戳
    if (t1 <= accBuf.back().first)
    {
        // 移除时间戳小于等于 t0 的IMU数据
        while (accBuf.front().first <= t0)
        {
            accBuf.pop();
            gyrBuf.pop();
        }
        // 将时间戳在 t0 与 t1 之间的IMU数据存入accVector和gyrVector中
        while (accBuf.front().first < t1)
        {
            accVector.push_back(accBuf.front());
            accBuf.pop();
            gyrVector.push_back(gyrBuf.front());
            gyrBuf.pop();
        }
        // 存入时间戳为 t1 的IMU数据
        accVector.push_back(accBuf.front());
        gyrVector.push_back(gyrBuf.front());
    }
    else
    {
        printf("wait for imu\n");
        return false;
    }
    return true;
}

// 这个函数看看能不能拿imu数据，只有当
bool Estimator::IMUAvailable(double t)
{
    // 只有当accBuf.empty()==false;也就是accBuf不是空的时候，而且传入的时间戳t小于等于accBuf的最后一个时间戳
    // 其余情况都返回false
    if (!accBuf.empty() && t <= accBuf.back().first)
        return true;
    else
        return false;
}

// Estimator 类中的 processMeasurements 方法用于处理测量数据
void Estimator::processMeasurements()
{
    while (1)
    {
        // printf("process measurments\n");
        //  从特征队列中获取特征数据
        // 这个里面的double代表的是时间戳，map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>代表的是特征帧数据
        pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>> feature;

        // 这两个变量存储的是加速度与角速度
        vector<pair<double, Eigen::Vector3d>> accVector, gyrVector;

        // 判断特征帧缓冲区是否存在值，如果存在数据的话就进行处理
        if (!featureBuf.empty())
        {
            // 获取缓冲区队列头的特征帧数据
            feature = featureBuf.front();

            // mark feature.first就是每一帧对应的时间戳，td是代表什么意思呢？
            // done 这个td是从配置文件config里面读取出来的，表示对时间戳进行一个时间矫正，这样变成一个真实的时间
            // mark 但是我没想通的地方是，为什么td初始化是0？？？
            curTime = feature.first + td;
            while (1)
            {
                // 如果不使用 IMU 或者 IMU 数据不可用，则跳出循环
                if ((!USE_IMU || IMUAvailable(feature.first + td)))
                    break;
                else
                {
                    printf("wait for imu ... \n");
                    // 如果不使用多线程，则直接返回
                    if (!MULTIPLE_THREAD)
                        return;
                    // 等待 5 毫秒
                    std::chrono::milliseconds dura(5);
                    std::this_thread::sleep_for(dura);
                }
            }
            mBuf.lock();
            // 如果使用 IMU 数据，则获取 IMU 数据
            if (USE_IMU)
                // 获取加速度与角速度到accVector, gyrVector，这里采用引用传递，函数内部的值变了，这个accVector, gyrVector跟着变
                getIMUInterval(prevTime, curTime, accVector, gyrVector);

            // 特征帧缓冲区第一个数据出队
            featureBuf.pop();
            mBuf.unlock();

            // 如果使用 IMU 数据，并且未初始化过姿态信息，初始化第一帧 IMU 姿态
            if (USE_IMU)
            {
                if (!initFirstPoseFlag)
                    // 因为加速度计在静止时理论上的值应该为（0，0，重力）；
                    // 故我们可以通过测量一段时间的加速度计信息取平均得到 averAcc；
                    // 然后计算 averAcc 到 理论值（0，0，重力）之间的旋转矩阵；
                    initFirstIMUPose(accVector);
                for (size_t i = 0; i < accVector.size(); i++)
                {
                    double dt;
                    if (i == 0)
                        dt = accVector[i].first - prevTime;
                    else if (i == accVector.size() - 1)
                        dt = curTime - accVector[i - 1].first;
                    else
                        dt = accVector[i].first - accVector[i - 1].first;
                    // IMU预积分函数
                    processIMU(accVector[i].first, dt, accVector[i].second, gyrVector[i].second);
                }
            }

            mProcess.lock();
            // 处理图像特征数据
            processImage(feature.second, feature.first);
            prevTime = curTime;

            // 打印统计信息
            printStatistics(*this, 0);

            std_msgs::Header header;
            header.frame_id = "world";
            header.stamp = ros::Time(feature.first);

            // 发布里程计数据
            pubOdometry(*this, header);
            // 发布关键位姿数据
            pubKeyPoses(*this, header);
            // 发布相机姿态数据
            pubCameraPose(*this, header);
            // 发布点云数据
            pubPointCloud(*this, header);
            // 发布关键帧数据
            pubKeyframe(*this);
            // 发布 TF 数据
            pubTF(*this, header);
            mProcess.unlock();
        }

        // 如果不使用多线程，则跳出循环
        if (!MULTIPLE_THREAD)
            break;

        // 等待 2 毫秒
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

// 初始化第一个IMU姿势
void Estimator::initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector)
{
    // 打印消息
    printf("init first imu pose\n");

    // 设置初始化标志为true
    initFirstPoseFlag = true;

    // 初始化平均加速度向量
    Eigen::Vector3d averAcc(0, 0, 0);

    // 获取加速度向量的大小
    int n = (int)accVector.size();

    // 遍历加速度向量
    for (size_t i = 0; i < accVector.size(); i++)
    {
        // 计算加速度向量的和
        averAcc = averAcc + accVector[i].second;
    }

    // 计算平均加速度
    averAcc = averAcc / n;

    // 打印平均加速度信息
    printf("averge acc %f %f %f\n", averAcc.x(), averAcc.y(), averAcc.z());

    // mark 为什么要用偏航角修正呢？？？
    // 计算对应的旋转矩阵
    Matrix3d R0 = Utility::g2R(averAcc);

    // 获取偏航角
    double yaw = Utility::R2ypr(R0).x();

    //  将翻滚角、府仰角设为零，重新求解旋转矩阵（应该是绕Z轴的旋转矩阵）
    // 修正旋转矩阵
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;

    // 存储旋转矩阵
    Rs[0] = R0;
    cout << "init R0 " << endl
         << Rs[0] << endl; // 打印旋转矩阵信息
    // Vs[0] = Vector3d(5, 0, 0);  // 设置速度状态量
}

// 初始化第一帧的位姿
void Estimator::initFirstPose(Eigen::Vector3d p, Eigen::Matrix3d r)
{
    Ps[0] = p;
    Rs[0] = r;
    initP = p;
    initR = r;
}

void Estimator::processIMU(double t, double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{
    // 检查 是不是第一个接受到IMU数据
    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }

    // 检查当前帧是不是已经有预积分的一些变量
    if (!pre_integrations[frame_count])
    {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }
    // 如果当前帧不是第一帧，就计算下面的，把预积分的一些数据保存好存到pre_integrations中
    if (frame_count != 0)
    {
        // 注意：这里的push_back被重写了，调用的是预积分对象中的push_back方法，
        // 在里面保存IMU消息，中值积分递推、更新雅可比矩阵、协方差矩阵
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
        // if(solver_flag != NON_LINEAR)

        // 临时预积分对象：中值积分递推、更新雅可比矩阵、协方差矩阵
        tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

        // 保存时间戳列表
        dt_buf[frame_count].push_back(dt);

        // 保存加速度与角加速度列表
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);

        // 中值积分，计算预积分（也就是变化量），与上一帧状态量不断累加，从而估算出当前帧的R、V、P;  作为后端优化的初始值；
        int j = frame_count;
        // 未校准的加速度与角速度
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;

        // mark IMU预积分当前时刻中值法的离散形式
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];

        // 似乎是根据未校准的角速度和时间增量 dt 来进行旋转矩阵的更新操作。
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

        // 对位置进行更新
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
        Vs[j] += dt * un_acc;
    }

    // 把当前时刻的加速度与角速度进行更新，在下一个dt时间段内进行更新
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

// 在Estimator类中的processImage方法里添加注释
void Estimator::processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const double header)
{
    ROS_DEBUG("new image coming ------------------------------------------"); // 记录调试信息：新图片到达
    ROS_DEBUG("Adding feature points %lu", image.size());                     // 记录调试信息：添加特征点数量

    // addFeatureCheckParallax函数解释
    //  对当前帧与之前帧进行视差比较，如果是当前帧变化很小，就会删去倒数第二帧，如果变化很大，就删去最旧的帧。并把这一帧作为新的关键帧
    //  这样也就保证了划窗内优化的,除了最后一帧可能不是关键帧外,其余的都是关键帧
    //  VINS里为了控制优化计算量，在实时情况下，只对当前帧之前某一部分帧进行优化，而不是全部历史帧。局部优化帧的数量就是窗口大小。
    //  为了维持窗口大小，需要去除旧的帧添加新的帧，也就是边缘化 Marginalization。到底是删去最旧的帧（MARGIN_OLD）还是删去刚
    //  刚进来窗口倒数第二帧(MARGIN_SECOND_NEW)

    // 判断之后,确定marg掉哪个帧
    // 若为关键帧，则在边缘化的时候marge掉最老的一帧；
    // 若不为关键帧，则在边缘化的时候marge掉次新帧；
    if (f_manager.addFeatureCheckParallax(frame_count, image, td)) // 调用f_manager的addFeatureCheckParallax方法
    {
        // 将边缘化标志设置为MARGIN_OLD
        marginalization_flag = MARGIN_OLD;
        // printf("keyframe\n");
    }
    else
    {
        // 将边缘化标志设置为MARGIN_SECOND_NEW
        marginalization_flag = MARGIN_SECOND_NEW;
        // printf("non-keyframe\n");
    }

    // 根据边缘化标志记录调试信息
    ROS_DEBUG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");

    // 记录调试信息：解算帧数
    ROS_DEBUG("Solving %d", frame_count);

    // 记录调试信息：特征点数量
    ROS_DEBUG("number of feature: %d", f_manager.getFeatureCount());

    // 保留时间戳，将Headers数组的第frame_count个
    Headers[frame_count] = header;

    /*构造新的一帧，加入all_image_frame*/
    // 创建ImageFrame对象
    ImageFrame imageframe(image, header);

    // 设置imageframe的预积分属性
    imageframe.pre_integration = tmp_pre_integration;

    // 将imageframe插入到all_image_frame中
    all_image_frame.insert(make_pair(header, imageframe));

    // 这里创建了一个临时的预积分对象
    // 创建IntegrationBase对象
    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};

    // 如果ESTIMATE_EXTRINSIC等于2
    if (ESTIMATE_EXTRINSIC == 2)
    {
        // 校准外参参数，需要进行旋转运动
        ROS_INFO("calibrating extrinsic param, rotation movement is needed");
        if (frame_count != 0) // 如果帧数不为0
        {
            // 这里面存储的是新图像与上一帧图像的特征点对(这里应该是需要有公共地图点才能配对)
            // 获取对应的特征点
            vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding(frame_count - 1, frame_count);

            // 创建Matrix3d对象calib_ric
            Matrix3d calib_ric;

            // 这里有这个函数CalibrationExRotation进行相机校准，如果校准成功的话，就输出校准信息
            // 并且将校准后的值给赋值给RIC
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric)) // 调用initial_ex_rotation的CalibrationExRotation方法
            {
                // 记录警告信息：初始外参旋转校准成功
                ROS_WARN("initial extrinsic rotation calib success");

                // 记录警告信息：初始外参旋转
                ROS_WARN_STREAM("initial extrinsic rotation: " << endl
                                                               << calib_ric);

                // 将ric数组的第一个元素设置为calib_ric
                ric[0] = calib_ric;

                // 将RIC数组的第一个元素设置为calib_ric
                RIC[0] = calib_ric;

                // 将ESTIMATE_EXTRINSIC设置为1
                ESTIMATE_EXTRINSIC = 1;
            }
        }
    }

    // 当求解器标志为INITIAL时执行以下代码
    // 初始的时候solver_flag就是INITIAL，这是对第一帧的操作，初始化
    if (solver_flag == INITIAL)
    {
        // 单目+IMU初始化
        if (!STEREO && USE_IMU)
        {
            // 若帧计数等于窗口大小，默认WINDOW_SIZE是10，也就是10帧为一个滑动窗口
            // 这里的意思是只有等到第10帧的时候才开始第一次优化
            if (frame_count == WINDOW_SIZE)
            {
                bool result = false;
                // 若ESTIMATE_EXTRINSIC不为2，这个表示有外参，并且（当前时间戳 - 初始时间戳）大于0.1
                // initial_timestamp初始化为0了
                if (ESTIMATE_EXTRINSIC != 2 && (header - initial_timestamp) > 0.1)
                {
                    // 执行初始化
                    result = initialStructure();

                    // 更新初始时间戳
                    initial_timestamp = header;
                }
                if (result)
                {
                    // 执行优化
                    optimization();

                    // 更新最新状态
                    updateLatestStates();

                    // 更新求解器标志为非线性
                    solver_flag = NON_LINEAR;

                    // 滑动窗口
                    slideWindow();

                    // 输出初始化完成信息
                    ROS_INFO("Initialization finish!");
                }
                else
                    // 滑动窗口
                    slideWindow();
            }
        }

        // 双目+IMU初始化
        if (STEREO && USE_IMU)
        {
            // 通过PnP初始化帧姿态
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);

            // 三角化
            f_manager.triangulate(frame_count, Ps, Rs, tic, ric);

            // 滑动窗口满了
            if (frame_count == WINDOW_SIZE)
            {
                // 使用迭代器遍历所有图像帧
                map<double, ImageFrame>::iterator frame_it;
                int i = 0;

                // 给滑窗内的帧设置R、t（IMU系到世界系）
                for (frame_it = all_image_frame.begin(); frame_it != all_image_frame.end(); frame_it++)
                {
                    // 更新图像帧旋转矩阵
                    frame_it->second.R = Rs[i];

                    // 更新图像帧平移向量
                    frame_it->second.T = Ps[i];
                    i++;
                }

                // 求解陀螺仪偏置 bg，重新计算预积分获取P、Q、V，和雅可比矩阵与协方差矩阵
                solveGyroscopeBias(all_image_frame, Bgs); // 解算陀螺仪偏置

                // 这个pre_integrations对象是estimator的没有与all_image_frame中的帧绑定，但是他们是一一对应的关系
                for (int i = 0; i <= WINDOW_SIZE; i++)
                {
                    pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]); // 重新积分
                }

                // 优化
                optimization();

                // 更新最新状态
                updateLatestStates();

                // 更新求解器标志为非线性
                solver_flag = NON_LINEAR;

                // 滑动窗口
                slideWindow();

                // 输出初始化完成信息
                ROS_INFO("Initialization finish!");
            }
        }

        // 双目初始化
        if (STEREO && !USE_IMU)
        {
            // 通过PnP初始化帧姿态
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);

            // 三角化
            f_manager.triangulate(frame_count, Ps, Rs, tic, ric);

            // 执行优化
            optimization();

            if (frame_count == WINDOW_SIZE)
            {
                // 执行优化
                optimization();

                // 更新最新状态
                updateLatestStates();

                // 更新求解器标志为非线性
                solver_flag = NON_LINEAR;

                // 滑动窗口
                slideWindow();

                // 输出初始化完成信息
                ROS_INFO("Initialization finish!");
            }
        }

        //    滑窗内帧不足时，位姿的传递（也就是将当前帧位姿传递给了下一帧，
        //    在processIMU中会利用这个传递的位姿+两帧之间的IMU信息累积递推出下一帧的位姿信息）
        if (frame_count < WINDOW_SIZE)
        {
            frame_count++; // 帧计数加一
            int prev_frame = frame_count - 1;
            Ps[frame_count] = Ps[prev_frame];   // 更新平移向量
            Vs[frame_count] = Vs[prev_frame];   // 更新速度
            Rs[frame_count] = Rs[prev_frame];   // 更新旋转矩阵
            Bas[frame_count] = Bas[prev_frame]; // 更新加速度计偏置
            Bgs[frame_count] = Bgs[prev_frame]; // 更新陀螺仪偏置
        }
    }

    // 如果不是第一帧，则执行以下操作
    else
    {
        // 计时求解器执行时间
        TicToc t_solve;
        // 若不使用IMU，则通过PnP初始化帧姿态
        if (!USE_IMU)
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
        // 三角化特征点
        f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
        // 优化
        optimization();
        // 移除索引
        set<int> removeIndex;
        // 进行异常值剔除
        outliersRejection(removeIndex);
        // 移除特征点异常值
        f_manager.removeOutlier(removeIndex);
        // 若不使用多线程
        if (!MULTIPLE_THREAD)
        {
            // 移除特征点跟踪异常值
            featureTracker.removeOutliers(removeIndex);
            // 预测下一帧的特征点位置
            predictPtsInNextFrame();
        }

        // 输出求解器耗时
        ROS_DEBUG("solver costs: %fms", t_solve.toc());

        // 进行失败检测
        if (failureDetection())
        {
            // 输出警告信息
            ROS_WARN("failure detection!");
            // 发生失败
            failure_occur = 1;
            // 清空状态
            clearState();
            // 设置参数
            setParameter();
            // 输出警告信息
            ROS_WARN("system reboot!");
            return;
        }

        // 滑动窗口
        slideWindow();
        // 移除失败的特征点
        f_manager.removeFailures();
        // 准备输出VINS的数据
        key_poses.clear();
        for (int i = 0; i <= WINDOW_SIZE; i++)
            key_poses.push_back(Ps[i]);

        // 更新最新的姿态
        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];
        updateLatestStates();
    }
}

bool Estimator::initialStructure()
{
    TicToc t_sfm;
    // check imu observibility
    // 检查IMU的可观测性
    {
        map<double, ImageFrame>::iterator frame_it;
        Vector3d sum_g;
        // 遍历所有的之前的图像帧(应该是包含当前帧)
        // 这里貌似是在计算加速度的和
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            sum_g += tmp_g;
        }
        Vector3d aver_g;

        // 计算平均加速度
        aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1);

        // 存储加速度方差的容器
        double var = 0;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
            // cout << "frame g " << tmp_g.transpose() << endl;
        }
        // 计算加速度的标准差
        var = sqrt(var / ((int)all_image_frame.size() - 1));
        // ROS_WARN("IMU variation %f!", var);

        // 如果加速度的标准差小于0.25，则输出一条信息表明IMU的激励不足
        if (var < 0.25)
        {
            ROS_INFO("IMU excitation not enouth!");
            // return false;
        }
    }
    // global sfm（Structure from Motion）
    // 从运动中重建出三维场景

    // 定义了一个名为 Q 的数组，用于存储帧之间的旋转信息，其中 frame_count 是帧的总数。
    Quaterniond Q[frame_count + 1];

    // 定义了一个名为 T 的数组，用于存储帧之间的平移信息。
    Vector3d T[frame_count + 1];

    // 定义了一个名为 sfm_tracked_points 的映射，用于存储SFM中跟踪的特征点的三维坐标。
    map<int, Vector3d> sfm_tracked_points;

    // 定义了一个名为 sfm_f 的向量，用于存储SFM中提取的特征点及其观测。
    vector<SFMFeature> sfm_f;

    // 对特征管理器中的每个特征进行遍历。
    // 实际上就是要搞清楚某一个地图点对应的哪些像素的特征点
    // 将f_manager中的所有feature保存到vector<SFMFeature> sfm_f中
    //     struct SFMFeature 其存放的是特征点的信息
    // {
    //     bool state;//状态（是否被三角化）
    //     int id;
    //     vector<pair<int,Vector2d>> observation;//所有观测到该地图点的图像帧ID和图像坐标
    //     double position[3];//3d坐标
    //     double depth;//深度
    // };
    for (auto &it_per_id : f_manager.feature)
    {
        // 获取特征的起始帧索引。
        int imu_j = it_per_id.start_frame - 1;

        // 定义了一个临时特征对象
        SFMFeature tmp_feature;
        tmp_feature.state = false;
        tmp_feature.id = it_per_id.feature_id;

        // 每一个地图点所对应的很多帧帧，每一帧进行遍历
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            // 获取特征在当前帧中的二维像素坐标。
            Vector3d pts_j = it_per_frame.point;

            // 将IMU帧索引和特征的二维观测值添加到临时特征的观测列表中。
            tmp_feature.observation.push_back(make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
        }
        sfm_f.push_back(tmp_feature);
    }

    // 定义了一个名为 relative_R 的3x3矩阵，用于存储相对旋转信息。
    Matrix3d relative_R;

    // 定义了一个名为 relative_T 的三维向量，用于存储相对平移信息。
    Vector3d relative_T;
    int l;

    // 调用一个函数 relativePose() 来计算相对姿态信息，如果计算失败则执行下一步。
    // 保证具有足够的视差,由E矩阵恢复R、t
    // 这里的第L帧是从第一帧开始到滑动窗口中第一个满足与当前帧的平均视差足够大的帧，会作为参考帧到下面的全局sfm使用，得到的Rt为当前帧到第l帧的坐标系变换Rt
    if (!relativePose(relative_R, relative_T, l))
    {
        ROS_INFO("Not enough features or parallax; Move device around");
        return false;
    }
    GlobalSFM sfm;
    if (!sfm.construct(frame_count + 1, Q, T, l,
                       relative_R, relative_T,
                       sfm_f, sfm_tracked_points))
    {
        ROS_DEBUG("global SFM failed!");
        marginalization_flag = MARGIN_OLD;
        return false;
    }

    // solve pnp for all frame
    map<double, ImageFrame>::iterator frame_it;
    map<int, Vector3d>::iterator it;
    frame_it = all_image_frame.begin();
    for (int i = 0; frame_it != all_image_frame.end(); frame_it++)
    {
        // provide initial guess
        cv::Mat r, rvec, t, D, tmp_r;
        if ((frame_it->first) == Headers[i])
        {
            frame_it->second.is_key_frame = true;
            frame_it->second.R = Q[i].toRotationMatrix() * RIC[0].transpose();
            frame_it->second.T = T[i];
            i++;
            continue;
        }
        if ((frame_it->first) > Headers[i])
        {
            i++;
        }
        Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
        Vector3d P_inital = -R_inital * T[i];
        cv::eigen2cv(R_inital, tmp_r);
        cv::Rodrigues(tmp_r, rvec);
        cv::eigen2cv(P_inital, t);

        frame_it->second.is_key_frame = false;
        vector<cv::Point3f> pts_3_vector;
        vector<cv::Point2f> pts_2_vector;
        for (auto &id_pts : frame_it->second.points)
        {
            int feature_id = id_pts.first;
            for (auto &i_p : id_pts.second)
            {
                it = sfm_tracked_points.find(feature_id);
                if (it != sfm_tracked_points.end())
                {
                    Vector3d world_pts = it->second;
                    cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
                    pts_3_vector.push_back(pts_3);
                    Vector2d img_pts = i_p.second.head<2>();
                    cv::Point2f pts_2(img_pts(0), img_pts(1));
                    pts_2_vector.push_back(pts_2);
                }
            }
        }
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
        if (pts_3_vector.size() < 6)
        {
            cout << "pts_3_vector size " << pts_3_vector.size() << endl;
            ROS_DEBUG("Not enough points for solve pnp !");
            return false;
        }
        if (!cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1))
        {
            ROS_DEBUG("solve pnp fail!");
            return false;
        }
        cv::Rodrigues(rvec, r);
        MatrixXd R_pnp, tmp_R_pnp;
        cv::cv2eigen(r, tmp_R_pnp);
        R_pnp = tmp_R_pnp.transpose();
        MatrixXd T_pnp;
        cv::cv2eigen(t, T_pnp);
        T_pnp = R_pnp * (-T_pnp);
        frame_it->second.R = R_pnp * RIC[0].transpose();
        frame_it->second.T = T_pnp;
    }
    if (visualInitialAlign())
        return true;
    else
    {
        ROS_INFO("misalign visual structure with IMU");
        return false;
    }
}

// 在函数级别添加注释
// Estimator 类的 visualInitialAlign 方法
bool Estimator::visualInitialAlign()
{
    // 计时器对象 t_g
    TicToc t_g;

    // 向量 x 用于存储结果
    VectorXd x;

    // 解决尺度问题
    // 调用 VisualIMUAlignment 方法，解决视觉IMU对准问题
    bool result = VisualIMUAlignment(all_image_frame, Bgs, g, x);
    if (!result) // 如果解决失败
    {
        ROS_DEBUG("solve g failed!"); // 输出调试信息
        return false;                 // 返回假值
    }

    // 更改状态
    // 循环处理每帧图像
    for (int i = 0; i <= frame_count; i++)
    {
        // 获取旋转矩阵
        Matrix3d Ri = all_image_frame[Headers[i]].R;

        // 获取平移向量
        Vector3d Pi = all_image_frame[Headers[i]].T;

        // 存储平移向量
        Ps[i] = Pi;

        // 存储旋转矩阵
        Rs[i] = Ri;

        // 设置为关键帧
        all_image_frame[Headers[i]].is_key_frame = true;
    }

    // 获取尺度参数
    double s = (x.tail<1>())(0);

    // 循环处理每个窗口
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        // 重新计算积分
        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
    }
    // 倒序处理每帧图像,更新平移向量
    for (int i = frame_count; i >= 0; i--)
        Ps[i] = s * Ps[i] - Rs[i] * TIC[0] - (s * Ps[0] - Rs[0] * TIC[0]);

    // 关键帧索引
    int kv = -1;

    // 迭代器
    map<double, ImageFrame>::iterator frame_i;

    // 遍历所有图像帧
    for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++)
    {
        // 如果是关键帧
        if (frame_i->second.is_key_frame)
        {
            // 关键帧索引自增
            kv++;

            // 计算速度
            Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3);
        }
    }

    // 根据重力向量得到旋转矩阵
    Matrix3d R0 = Utility::g2R(g);

    // 计算偏航角
    double yaw = Utility::R2ypr(R0 * Rs[0]).x();

    // 更新旋转矩阵
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;

    // 更新重力向量
    g = R0 * g;

    // 计算旋转差异
    Matrix3d rot_diff = R0;

    // 循环处理每帧
    for (int i = 0; i <= frame_count; i++)
    {
        Ps[i] = rot_diff * Ps[i]; // 更新平移向量
        Rs[i] = rot_diff * Rs[i]; // 更新旋转矩阵
        Vs[i] = rot_diff * Vs[i]; // 更新速度
    }

    // 输出调试信息
    ROS_DEBUG_STREAM("g0     " << g.transpose());

    // 输出调试信息
    ROS_DEBUG_STREAM("my R0  " << Utility::R2ypr(Rs[0]).transpose());

    // 清除深度信息
    f_manager.clearDepth();

    // 三角化重构特征点
    f_manager.triangulate(frame_count, Ps, Rs, tic, ric);

    // 返回真值
    return true;
}

// Estimator 类中的 relativePose 方法用于寻找相对位姿
// 这个函数的目标是在窗口内寻找与最新帧具有足够匹配点和视差的先前帧
bool Estimator::relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l)
{
    // 寻找包含足够匹配和最新帧有足够视差的先前帧
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        vector<pair<Vector3d, Vector3d>> corres;

        // 对于每一个先前的帧，获取与最新帧的匹配点对儿
        corres = f_manager.getCorresponding(i, WINDOW_SIZE);

        // 如果匹配点数大于20
        if (corres.size() > 20)
        {
            double sum_parallax = 0;
            double average_parallax;
            for (int j = 0; j < int(corres.size()); j++)
            {
                // 计算所有匹配点对的视差（平均3D点之间的距离）。
                Vector2d pts_0(corres[j].first(0), corres[j].first(1));
                Vector2d pts_1(corres[j].second(0), corres[j].second(1));

                // 计算视差
                // 相机的不同位置拍摄的图像中同一个物体在像素坐标上的差异。
                // 简而言之就是地图点在像素平面投影的差异
                double parallax = (pts_0 - pts_1).norm();
                sum_parallax = sum_parallax + parallax;
            }

            // 计算平均视差
            average_parallax = 1.0 * sum_parallax / int(corres.size());

            // 如果平均视差乘以460大于30，并且使用匹配点对求解相对旋转矩阵和平移向量成功
            if (average_parallax * 460 > 30 && m_estimator.solveRelativeRT(corres, relative_R, relative_T))
            {
                l = i; // 设置 l 为当前帧
                ROS_DEBUG("average_parallax %f choose l %d and newest frame to triangulate the whole structure", average_parallax * 460, l);
                return true; // 返回 true 表示成功找到符合条件的帧
            }
        }
    }
    return false; // 未找到符合条件的帧，返回 false
}

// 在执行优化的时候vector2double()函数将优化变量转化为double数组
// 这个是双精度浮点数，放便优化
// 双精度浮点数这样精度比较高？
void Estimator::vector2double()
{
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        para_Pose[i][0] = Ps[i].x();
        para_Pose[i][1] = Ps[i].y();
        para_Pose[i][2] = Ps[i].z();
        Quaterniond q{Rs[i]};
        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();

        if (USE_IMU)
        {
            para_SpeedBias[i][0] = Vs[i].x();
            para_SpeedBias[i][1] = Vs[i].y();
            para_SpeedBias[i][2] = Vs[i].z();

            para_SpeedBias[i][3] = Bas[i].x();
            para_SpeedBias[i][4] = Bas[i].y();
            para_SpeedBias[i][5] = Bas[i].z();

            para_SpeedBias[i][6] = Bgs[i].x();
            para_SpeedBias[i][7] = Bgs[i].y();
            para_SpeedBias[i][8] = Bgs[i].z();
        }
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        para_Ex_Pose[i][0] = tic[i].x();
        para_Ex_Pose[i][1] = tic[i].y();
        para_Ex_Pose[i][2] = tic[i].z();
        Quaterniond q{ric[i]};
        para_Ex_Pose[i][3] = q.x();
        para_Ex_Pose[i][4] = q.y();
        para_Ex_Pose[i][5] = q.z();
        para_Ex_Pose[i][6] = q.w();
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        para_Feature[i][0] = dep(i);

    para_Td[0][0] = td;
}

void Estimator::double2vector()
{
    Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
    Vector3d origin_P0 = Ps[0];

    if (failure_occur)
    {
        origin_R0 = Utility::R2ypr(last_R0);
        origin_P0 = last_P0;
        failure_occur = 0;
    }

    if (USE_IMU)
    {
        Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Pose[0][6],
                                                         para_Pose[0][3],
                                                         para_Pose[0][4],
                                                         para_Pose[0][5])
                                                 .toRotationMatrix());
        double y_diff = origin_R0.x() - origin_R00.x();
        // TODO
        Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
        if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0)
        {
            ROS_DEBUG("euler singular point!");
            rot_diff = Rs[0] * Quaterniond(para_Pose[0][6],
                                           para_Pose[0][3],
                                           para_Pose[0][4],
                                           para_Pose[0][5])
                                   .toRotationMatrix()
                                   .transpose();
        }

        for (int i = 0; i <= WINDOW_SIZE; i++)
        {

            Rs[i] = rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();

            Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                        para_Pose[i][1] - para_Pose[0][1],
                                        para_Pose[i][2] - para_Pose[0][2]) +
                    origin_P0;

            Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0],
                                        para_SpeedBias[i][1],
                                        para_SpeedBias[i][2]);

            Bas[i] = Vector3d(para_SpeedBias[i][3],
                              para_SpeedBias[i][4],
                              para_SpeedBias[i][5]);

            Bgs[i] = Vector3d(para_SpeedBias[i][6],
                              para_SpeedBias[i][7],
                              para_SpeedBias[i][8]);
        }
    }
    else
    {
        for (int i = 0; i <= WINDOW_SIZE; i++)
        {
            Rs[i] = Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();

            Ps[i] = Vector3d(para_Pose[i][0], para_Pose[i][1], para_Pose[i][2]);
        }
    }

    if (USE_IMU)
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            tic[i] = Vector3d(para_Ex_Pose[i][0],
                              para_Ex_Pose[i][1],
                              para_Ex_Pose[i][2]);
            ric[i] = Quaterniond(para_Ex_Pose[i][6],
                                 para_Ex_Pose[i][3],
                                 para_Ex_Pose[i][4],
                                 para_Ex_Pose[i][5])
                         .normalized()
                         .toRotationMatrix();
        }
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        dep(i) = para_Feature[i][0];
    f_manager.setDepth(dep);

    if (USE_IMU)
        td = para_Td[0][0];
}

bool Estimator::failureDetection()
{
    return false;
    if (f_manager.last_track_num < 2)
    {
        ROS_INFO(" little feature %d", f_manager.last_track_num);
        // return true;
    }
    if (Bas[WINDOW_SIZE].norm() > 2.5)
    {
        ROS_INFO(" big IMU acc bias estimation %f", Bas[WINDOW_SIZE].norm());
        return true;
    }
    if (Bgs[WINDOW_SIZE].norm() > 1.0)
    {
        ROS_INFO(" big IMU gyr bias estimation %f", Bgs[WINDOW_SIZE].norm());
        return true;
    }
    /*
    if (tic(0) > 1)
    {
        ROS_INFO(" big extri param estimation %d", tic(0) > 1);
        return true;
    }
    */
    Vector3d tmp_P = Ps[WINDOW_SIZE];
    if ((tmp_P - last_P).norm() > 5)
    {
        // ROS_INFO(" big translation");
        // return true;
    }
    if (abs(tmp_P.z() - last_P.z()) > 1)
    {
        // ROS_INFO(" big z translation");
        // return true;
    }
    Matrix3d tmp_R = Rs[WINDOW_SIZE];
    Matrix3d delta_R = tmp_R.transpose() * last_R;
    Quaterniond delta_Q(delta_R);
    double delta_angle;
    delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
    if (delta_angle > 50)
    {
        ROS_INFO(" big delta_angle ");
        // return true;
    }
    return false;
}

// 状态向量共包括滑动窗口内的 n+1 个所有相机的状态（包括位置、旋转、速度、加速度计 bias 和陀螺仪 bias）
// 、Camera 到 IMU 的外参、m+1 个 3D 点的逆深度：
// 这个函数可太重要了，也是后端优化的核心部分，大概包括了300多行代码
void Estimator::optimization()
{
    TicToc t_whole, t_prepare;
    vector2double();

    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    // loss_function = NULL;
    loss_function = new ceres::HuberLoss(1.0);
    // loss_function = new ceres::CauchyLoss(1.0 / FOCAL_LENGTH);
    // ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
    for (int i = 0; i < frame_count + 1; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
        if (USE_IMU)
            problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
    }
    if (!USE_IMU)
        problem.SetParameterBlockConstant(para_Pose[0]);

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        if ((ESTIMATE_EXTRINSIC && frame_count == WINDOW_SIZE && Vs[0].norm() > 0.2) || openExEstimation)
        {
            // ROS_INFO("estimate extinsic param");
            openExEstimation = 1;
        }
        else
        {
            // ROS_INFO("fix extinsic param");
            problem.SetParameterBlockConstant(para_Ex_Pose[i]);
        }
    }
    problem.AddParameterBlock(para_Td[0], 1);

    if (!ESTIMATE_TD || Vs[0].norm() < 0.2)
        problem.SetParameterBlockConstant(para_Td[0]);

    if (last_marginalization_info && last_marginalization_info->valid)
    {
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        problem.AddResidualBlock(marginalization_factor, NULL,
                                 last_marginalization_parameter_blocks);
    }
    if (USE_IMU)
    {
        for (int i = 0; i < frame_count; i++)
        {
            int j = i + 1;
            if (pre_integrations[j]->sum_dt > 10.0)
                continue;
            IMUFactor *imu_factor = new IMUFactor(pre_integrations[j]);
            problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
        }
    }

    int f_m_cnt = 0;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;

        ++feature_index;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        Vector3d pts_i = it_per_id.feature_per_frame[0].point;

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i != imu_j)
            {
                Vector3d pts_j = it_per_frame.point;
                ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]);
            }

            if (STEREO && it_per_frame.is_stereo)
            {
                Vector3d pts_j_right = it_per_frame.pointRight;
                if (imu_i != imu_j)
                {
                    ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                                           it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
                }
                else
                {
                    ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                                           it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    problem.AddResidualBlock(f, loss_function, para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
                }
            }
            f_m_cnt++;
        }
    }

    ROS_DEBUG("visual measurement count: %d", f_m_cnt);
    // printf("prepare for ceres: %f \n", t_prepare.toc());

    ceres::Solver::Options options;

    options.linear_solver_type = ceres::DENSE_SCHUR;
    // options.num_threads = 2;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    // options.use_explicit_schur_complement = true;
    // options.minimizer_progress_to_stdout = true;
    // options.use_nonmonotonic_steps = true;
    if (marginalization_flag == MARGIN_OLD)
        options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
    else
        options.max_solver_time_in_seconds = SOLVER_TIME;
    TicToc t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    // cout << summary.BriefReport() << endl;
    ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
    // printf("solver costs: %f \n", t_solver.toc());

    double2vector();
    // printf("frame_count: %d \n", frame_count);

    if (frame_count < WINDOW_SIZE)
        return;

    TicToc t_whole_marginalization;
    if (marginalization_flag == MARGIN_OLD)
    {
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        vector2double();

        if (last_marginalization_info && last_marginalization_info->valid)
        {
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
            {
                if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
                    last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
                    drop_set.push_back(i);
            }
            // construct new marginlization_factor
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                           last_marginalization_parameter_blocks,
                                                                           drop_set);
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        if (USE_IMU)
        {
            if (pre_integrations[1]->sum_dt < 10.0)
            {
                IMUFactor *imu_factor = new IMUFactor(pre_integrations[1]);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                                                                               vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1], para_SpeedBias[1]},
                                                                               vector<int>{0, 1});
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }

        {
            int feature_index = -1;
            for (auto &it_per_id : f_manager.feature)
            {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (it_per_id.used_num < 4)
                    continue;

                ++feature_index;

                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                if (imu_i != 0)
                    continue;

                Vector3d pts_i = it_per_id.feature_per_frame[0].point;

                for (auto &it_per_frame : it_per_id.feature_per_frame)
                {
                    imu_j++;
                    if (imu_i != imu_j)
                    {
                        Vector3d pts_j = it_per_frame.point;
                        ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                                                  it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f_td, loss_function,
                                                                                       vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]},
                                                                                       vector<int>{0, 3});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                    if (STEREO && it_per_frame.is_stereo)
                    {
                        Vector3d pts_j_right = it_per_frame.pointRight;
                        if (imu_i != imu_j)
                        {
                            ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                                                   it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                           vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},
                                                                                           vector<int>{0, 4});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                        else
                        {
                            ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                                                   it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                           vector<double *>{para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},
                                                                                           vector<int>{2});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                    }
                }
            }
        }

        TicToc t_pre_margin;
        marginalization_info->preMarginalize();
        ROS_DEBUG("pre marginalization %f ms", t_pre_margin.toc());

        TicToc t_margin;
        marginalization_info->marginalize();
        ROS_DEBUG("marginalization %f ms", t_margin.toc());

        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++)
        {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
            if (USE_IMU)
                addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
        }
        for (int i = 0; i < NUM_OF_CAM; i++)
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

        addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];

        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

        if (last_marginalization_info)
            delete last_marginalization_info;
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;
    }
    else
    {
        if (last_marginalization_info &&
            std::count(std::begin(last_marginalization_parameter_blocks), std::end(last_marginalization_parameter_blocks), para_Pose[WINDOW_SIZE - 1]))
        {

            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            vector2double();
            if (last_marginalization_info && last_marginalization_info->valid)
            {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
                {
                    ROS_ASSERT(last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);
                    if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
                        drop_set.push_back(i);
                }
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                               last_marginalization_parameter_blocks,
                                                                               drop_set);

                marginalization_info->addResidualBlockInfo(residual_block_info);
            }

            TicToc t_pre_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->preMarginalize();
            ROS_DEBUG("end pre marginalization, %f ms", t_pre_margin.toc());

            TicToc t_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->marginalize();
            ROS_DEBUG("end marginalization, %f ms", t_margin.toc());

            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                if (i == WINDOW_SIZE - 1)
                    continue;
                else if (i == WINDOW_SIZE)
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    if (USE_IMU)
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                }
                else
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                    if (USE_IMU)
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                }
            }
            for (int i = 0; i < NUM_OF_CAM; i++)
                addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

            addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];

            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
            if (last_marginalization_info)
                delete last_marginalization_info;
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;
        }
    }
    // printf("whole marginalization costs: %f \n", t_whole_marginalization.toc());
    // printf("whole time for ceres: %f \n", t_whole.toc());
}

void Estimator::slideWindow()
{
    TicToc t_margin;
    if (marginalization_flag == MARGIN_OLD)
    {
        double t_0 = Headers[0];
        back_R0 = Rs[0];
        back_P0 = Ps[0];
        if (frame_count == WINDOW_SIZE)
        {
            for (int i = 0; i < WINDOW_SIZE; i++)
            {
                Headers[i] = Headers[i + 1];
                Rs[i].swap(Rs[i + 1]);
                Ps[i].swap(Ps[i + 1]);
                if (USE_IMU)
                {
                    std::swap(pre_integrations[i], pre_integrations[i + 1]);

                    dt_buf[i].swap(dt_buf[i + 1]);
                    linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
                    angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);

                    Vs[i].swap(Vs[i + 1]);
                    Bas[i].swap(Bas[i + 1]);
                    Bgs[i].swap(Bgs[i + 1]);
                }
            }
            Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
            Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
            Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];

            if (USE_IMU)
            {
                Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
                Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
                Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];

                delete pre_integrations[WINDOW_SIZE];
                pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

                dt_buf[WINDOW_SIZE].clear();
                linear_acceleration_buf[WINDOW_SIZE].clear();
                angular_velocity_buf[WINDOW_SIZE].clear();
            }

            if (true || solver_flag == INITIAL)
            {
                map<double, ImageFrame>::iterator it_0;
                it_0 = all_image_frame.find(t_0);
                delete it_0->second.pre_integration;
                all_image_frame.erase(all_image_frame.begin(), it_0);
            }
            slideWindowOld();
        }
    }
    else
    {
        if (frame_count == WINDOW_SIZE)
        {
            Headers[frame_count - 1] = Headers[frame_count];
            Ps[frame_count - 1] = Ps[frame_count];
            Rs[frame_count - 1] = Rs[frame_count];

            if (USE_IMU)
            {
                for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++)
                {
                    double tmp_dt = dt_buf[frame_count][i];
                    Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
                    Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];

                    pre_integrations[frame_count - 1]->push_back(tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);

                    dt_buf[frame_count - 1].push_back(tmp_dt);
                    linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
                    angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
                }

                Vs[frame_count - 1] = Vs[frame_count];
                Bas[frame_count - 1] = Bas[frame_count];
                Bgs[frame_count - 1] = Bgs[frame_count];

                delete pre_integrations[WINDOW_SIZE];
                pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

                dt_buf[WINDOW_SIZE].clear();
                linear_acceleration_buf[WINDOW_SIZE].clear();
                angular_velocity_buf[WINDOW_SIZE].clear();
            }
            slideWindowNew();
        }
    }
}

void Estimator::slideWindowNew()
{
    sum_of_front++;
    f_manager.removeFront(frame_count);
}

void Estimator::slideWindowOld()
{
    sum_of_back++;

    bool shift_depth = solver_flag == NON_LINEAR ? true : false;
    if (shift_depth)
    {
        Matrix3d R0, R1;
        Vector3d P0, P1;
        R0 = back_R0 * ric[0];
        R1 = Rs[0] * ric[0];
        P0 = back_P0 + back_R0 * tic[0];
        P1 = Ps[0] + Rs[0] * tic[0];
        f_manager.removeBackShiftDepth(R0, P0, R1, P1);
    }
    else
        f_manager.removeBack();
}

void Estimator::getPoseInWorldFrame(Eigen::Matrix4d &T)
{
    T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = Rs[frame_count];
    T.block<3, 1>(0, 3) = Ps[frame_count];
}

void Estimator::getPoseInWorldFrame(int index, Eigen::Matrix4d &T)
{
    T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = Rs[index];
    T.block<3, 1>(0, 3) = Ps[index];
}

void Estimator::predictPtsInNextFrame()
{
    // printf("predict pts in next frame\n");
    if (frame_count < 2)
        return;
    // predict next pose. Assume constant velocity motion
    Eigen::Matrix4d curT, prevT, nextT;
    getPoseInWorldFrame(curT);
    getPoseInWorldFrame(frame_count - 1, prevT);
    nextT = curT * (prevT.inverse() * curT);
    map<int, Eigen::Vector3d> predictPts;

    for (auto &it_per_id : f_manager.feature)
    {
        if (it_per_id.estimated_depth > 0)
        {
            int firstIndex = it_per_id.start_frame;
            int lastIndex = it_per_id.start_frame + it_per_id.feature_per_frame.size() - 1;
            // printf("cur frame index  %d last frame index %d\n", frame_count, lastIndex);
            if ((int)it_per_id.feature_per_frame.size() >= 2 && lastIndex == frame_count)
            {
                double depth = it_per_id.estimated_depth;
                Vector3d pts_j = ric[0] * (depth * it_per_id.feature_per_frame[0].point) + tic[0];
                Vector3d pts_w = Rs[firstIndex] * pts_j + Ps[firstIndex];
                Vector3d pts_local = nextT.block<3, 3>(0, 0).transpose() * (pts_w - nextT.block<3, 1>(0, 3));
                Vector3d pts_cam = ric[0].transpose() * (pts_local - tic[0]);
                int ptsIndex = it_per_id.feature_id;
                predictPts[ptsIndex] = pts_cam;
            }
        }
    }
    featureTracker.setPrediction(predictPts);
    // printf("estimator output %d predict pts\n",(int)predictPts.size());
}

double Estimator::reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici,
                                    Matrix3d &Rj, Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj,
                                    double depth, Vector3d &uvi, Vector3d &uvj)
{
    Vector3d pts_w = Ri * (rici * (depth * uvi) + tici) + Pi;
    Vector3d pts_cj = ricj.transpose() * (Rj.transpose() * (pts_w - Pj) - ticj);
    Vector2d residual = (pts_cj / pts_cj.z()).head<2>() - uvj.head<2>();
    double rx = residual.x();
    double ry = residual.y();
    return sqrt(rx * rx + ry * ry);
}

void Estimator::outliersRejection(set<int> &removeIndex)
{
    // return;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature)
    {
        double err = 0;
        int errCnt = 0;
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;
        feature_index++;
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point;
        double depth = it_per_id.estimated_depth;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i != imu_j)
            {
                Vector3d pts_j = it_per_frame.point;
                double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0],
                                                     Rs[imu_j], Ps[imu_j], ric[0], tic[0],
                                                     depth, pts_i, pts_j);
                err += tmp_error;
                errCnt++;
                // printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
            }
            // need to rewrite projecton factor.........
            if (STEREO && it_per_frame.is_stereo)
            {

                Vector3d pts_j_right = it_per_frame.pointRight;
                if (imu_i != imu_j)
                {
                    double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0],
                                                         Rs[imu_j], Ps[imu_j], ric[1], tic[1],
                                                         depth, pts_i, pts_j_right);
                    err += tmp_error;
                    errCnt++;
                    // printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
                }
                else
                {
                    double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0],
                                                         Rs[imu_j], Ps[imu_j], ric[1], tic[1],
                                                         depth, pts_i, pts_j_right);
                    err += tmp_error;
                    errCnt++;
                    // printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
                }
            }
        }
        double ave_err = err / errCnt;
        if (ave_err * FOCAL_LENGTH > 3)
            removeIndex.insert(it_per_id.feature_id);
    }
}

// 这段代码的功能是根据传感器数据和之前的状态，通过一系列的运算和更新，进行姿态预测和状态估计，包括更新姿态、位置和速度等变量。
// 传入了线加速度，角速度以及当前的时间戳
void Estimator::fastPredictIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity)
{
    // latest_time为上一次状态更新的时间，这里计算时间差
    double dt = t - latest_time;
    latest_time = t;
    Eigen::Vector3d un_acc_0 = latest_Q * (latest_acc_0 - latest_Ba) - g;
    Eigen::Vector3d un_gyr = 0.5 * (latest_gyr_0 + angular_velocity) - latest_Bg;
    latest_Q = latest_Q * Utility::deltaQ(un_gyr * dt);
    Eigen::Vector3d un_acc_1 = latest_Q * (linear_acceleration - latest_Ba) - g;
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    latest_P = latest_P + dt * latest_V + 0.5 * dt * dt * un_acc;
    latest_V = latest_V + dt * un_acc;
    latest_acc_0 = linear_acceleration;
    latest_gyr_0 = angular_velocity;
}

// 主要功能是在多线程环境下更新最新的状态信息。它先获取互斥锁确保状态更新期间的数据一致性，
// 然后根据当前帧的数据更新最新的时间戳、位置、姿态、速度、加速度偏置、陀螺仪偏置和初始传感器数据。
// 接下来，它从加速度和陀螺仪缓冲区中读取数据，并调用fastPredictIMU()函数进行快速预测，更新预测的状态。
// 最后，释放互斥锁，完成状态更新过程。
void Estimator::updateLatestStates()
{
    mPropagate.lock();
    latest_time = Headers[frame_count] + td;
    latest_P = Ps[frame_count];
    latest_Q = Rs[frame_count];
    latest_V = Vs[frame_count];
    latest_Ba = Bas[frame_count];
    latest_Bg = Bgs[frame_count];
    latest_acc_0 = acc_0;
    latest_gyr_0 = gyr_0;
    mBuf.lock();
    queue<pair<double, Eigen::Vector3d>> tmp_accBuf = accBuf;
    queue<pair<double, Eigen::Vector3d>> tmp_gyrBuf = gyrBuf;
    mBuf.unlock();
    while (!tmp_accBuf.empty())
    {
        double t = tmp_accBuf.front().first;
        Eigen::Vector3d acc = tmp_accBuf.front().second;
        Eigen::Vector3d gyr = tmp_gyrBuf.front().second;
        fastPredictIMU(t, acc, gyr);
        tmp_accBuf.pop();
        tmp_gyrBuf.pop();
    }
    mPropagate.unlock();
}
