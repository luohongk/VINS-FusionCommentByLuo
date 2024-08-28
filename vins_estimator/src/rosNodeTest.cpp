/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

// 包含必要的头文件
#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/visualization.h"

// 创建估计器对象
Estimator estimator;

// 定义传感器消息队列和互斥锁
std::queue<sensor_msgs::ImuConstPtr> imu_buf;
std::queue<sensor_msgs::PointCloudConstPtr> feature_buf;
std::queue<sensor_msgs::ImageConstPtr> img0_buf;
std::queue<sensor_msgs::ImageConstPtr> img1_buf;
std::mutex m_buf;

// 图像回调函数，在此函数中将图像消息加入图像消息队列
void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    // 对消息队列加锁，保证线程安全
    m_buf.lock();
    img0_buf.push(img_msg);
    m_buf.unlock();
}
// 定义一个名为img1_callback的函数，用于处理接收到的图像信息
void img1_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();           // 锁定互斥体，以确保线程安全
    img1_buf.push(img_msg); // 将接收到的图像信息压入img1_buf队列中
    m_buf.unlock();         // 解锁互斥体，允许其他线程访问共享资源
}
// 从ROS消息中获取图像并转换为OpenCV格式的图像
cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    // 声明一个指向cv_bridge::CvImage类型的指针
    cv_bridge::CvImageConstPtr ptr;

    // 检查ROS消息的编码格式，如果为"8UC1"则进行处理，否则直接转换
    if (img_msg->encoding == "8UC1")
    {
        // 创建一个新的sensor_msgs::Image对象，并从输入的img_msg中复制相关信息
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";

        // 将新创建的img对象转换为OpenCV格式的图像
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        // 将输入的img_msg直接转换为OpenCV格式的图像
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    // 从指针中获取图像并进行克隆，返回克隆后的图像
    cv::Mat img = ptr->image.clone();
    return img;
}

// 从两个话题中提取具有相同时间戳的图像
void sync_process()
{
    while (1)
    {
        // 如果使用双目相机
        if (STEREO)
        {
            // 用于存储图像数据
            cv::Mat image0, image1;
            // 存储图像消息的头信息
            std_msgs::Header header;
            // 存储时间戳
            double time = 0;
            // 加锁以访问图像缓冲区
            m_buf.lock();
            // 这部分进行数据对齐，将双目相机的图像对齐到同一时间戳
            if (!img0_buf.empty() && !img1_buf.empty())
            {
                // 获取图像1的时间戳
                double time0 = img0_buf.front()->header.stamp.toSec();
                // 获取图像2的时间戳
                double time1 = img1_buf.front()->header.stamp.toSec();
                // 0.003秒的时间容差
                // 如果图像1比图像2早超过了0.003秒
                if (time0 < time1 - 0.003)
                {
                    // 丢弃图像1
                    img0_buf.pop();
                    // 打印丢弃图像1的消息
                    printf("throw img0\n");
                }
                // 如果图像1比图像2晚超过了0.003秒
                else if (time0 > time1 + 0.003) 
                {
                    // 丢弃图像2
                    img1_buf.pop();
                    // 打印丢弃图像2的消息
                    printf("throw img1\n");
                }
                // 如果图像1和图像2的时间戳在容差范围内
                else
                {
                    // 使用图像1的时间戳
                    time = img0_buf.front()->header.stamp.toSec();
                    // 获取图像1的头信息
                    header = img0_buf.front()->header;
                    // 获取图像1的数据
                    image0 = getImageFromMsg(img0_buf.front());
                    // 弹出图像1 
                    img0_buf.pop();
                    // 获取图像2的数据
                    image1 = getImageFromMsg(img1_buf.front());
                    // 弹出图像2
                    img1_buf.pop();
                    // printf("find img0 and img1\n");
                }
            }
            m_buf.unlock();                                 // 解锁图像缓冲区
            if (!image0.empty())                            // 如果图像1不为空
                estimator.inputImage(time, image0, image1); // 将图像数据传递给评估器进行处理
        }

        // 如果使用的是单目相机
        else
        {
            cv::Mat image;           // 用于存储图像数据
            std_msgs::Header header; // 存储图像消息的头信息
            double time = 0;         // 存储时间戳
            m_buf.lock();            // 加锁以访问图像缓冲区

            // 单目相机直接获取时间戳和图像数据，header数据
            if (!img0_buf.empty())
            {
                time = img0_buf.front()->header.stamp.toSec(); // 获取图像的时间戳
                header = img0_buf.front()->header;             // 获取图像的头信息
                image = getImageFromMsg(img0_buf.front());     // 获取图像的数据
                img0_buf.pop();                                // 弹出图像
            }
            m_buf.unlock();                     // 解锁图像缓冲区

            // inputImage为重要函数
            if (!image.empty())
                estimator.inputImage(time, image); // 将图像数据传递给评估器进行处理
        }

        std::chrono::milliseconds dura(2); // 定义时间间隔为2毫秒
        std::this_thread::sleep_for(dura); // 休眠2毫秒
    }
}

// 传感器回调函数，处理IMU数据
void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    // 从IMU消息中获取时间戳
    double t = imu_msg->header.stamp.toSec();
    // 获取线性加速度数据
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    // 获取角速度数据
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    // 将线性加速度数据和角速度数据存储为向量
    Vector3d acc(dx, dy, dz);
    Vector3d gyr(rx, ry, rz);
    // 将IMU数据输入到estimator中进行处理
    estimator.inputIMU(t, acc, gyr);
    // 返回
    return;
}

// 定义了一个名为 feature_callback 的回调函数，用来处理传感器消息中的特征数据
void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    // 创建了一个名为 featureFrame 的映射，用来存储特征数据
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    // 遍历特征消息中的点
    for (unsigned int i = 0; i < feature_msg->points.size(); i++)
    {
        // 获取特征的 id 和相机的 id
        int feature_id = feature_msg->channels[0].values[i];
        int camera_id = feature_msg->channels[1].values[i];
        // 获取特征点的坐标和像素坐标
        double x = feature_msg->points[i].x;
        double y = feature_msg->points[i].y;
        double z = feature_msg->points[i].z;
        double p_u = feature_msg->channels[2].values[i];
        double p_v = feature_msg->channels[3].values[i];
        // 获取特征点的速度信息
        double velocity_x = feature_msg->channels[4].values[i];
        double velocity_y = feature_msg->channels[5].values[i];
        // 如果特征消息中包含更多数据，则获取真实世界中的坐标信息并存储
        if (feature_msg->channels.size() > 5)
        {
            double gx = feature_msg->channels[6].values[i];
            double gy = feature_msg->channels[7].values[i];
            double gz = feature_msg->channels[8].values[i];
            pts_gt[feature_id] = Eigen::Vector3d(gx, gy, gz);
            // printf("receive pts gt %d %f %f %f\n", feature_id, gx, gy, gz);
        }
        // 断言 z 值为 1
        ROS_ASSERT(z == 1);
        // 创建包含特征点信息的 Eigen 矩阵
        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        // 将特征数据存储到 featureFrame 中
        featureFrame[feature_id].emplace_back(camera_id, xyz_uv_velocity);
    }
    // 获取特征消息的时间戳
    double t = feature_msg->header.stamp.toSec();
    // 将特征数据传递给 estimator 类处理
    estimator.inputFeature(t, featureFrame);
    return;
}

// 定义了一个名为 restart_callback 的函数，接收类型为 std_msgs::Bool 的指针参数 restart_msg
void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    // 如果 restart_msg 的数据为 true
    if (restart_msg->data == true)
    {
        // 输出警告信息
        ROS_WARN("restart the estimator!");
        // 清除估计器状态
        estimator.clearState();
        // 设置估计器参数
        estimator.setParameter();
    }
    // 返回
    return;
}

// imu_switch_callback函数：处理IMU开关状态的回调函数
void imu_switch_callback(const std_msgs::BoolConstPtr &switch_msg)
{
    if (switch_msg->data == true) // 如果开关状态为真
    {
        // ROS_WARN("use IMU!");  // 记录ROS警告：使用IMU！
        estimator.changeSensorType(1, STEREO); // 切换为使用IMU的传感器类型
    }
    else // 如果开关状态为假
    {
        // ROS_WARN("disable IMU!");  // 记录ROS警告：禁用IMU！
        estimator.changeSensorType(0, STEREO); // 切换为禁用IMU的传感器类型
    }
    return; // 返回
}

// 以回调函数的方式处理摄像头切换消息
void cam_switch_callback(const std_msgs::BoolConstPtr &switch_msg)
{
    if (switch_msg->data == true)
    {
        // 如果接收到的消息为真，切换到使用惯性测量单元和立体摄像头的状态
        estimator.changeSensorType(USE_IMU, 1);
    }
    else
    {
        // 如果接收到的消息为假，切换到使用惯性测量单元和单目摄像头（左侧）的状态
        estimator.changeSensorType(USE_IMU, 0);
    }
    return;
}

// 主函数，程序入口
int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "vins_estimator");
    // 创建节点句柄
    ros::NodeHandle n("~");
    // 设置ROS日志级别为Info
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    // 检查命令行参数个数是否为2
    if (argc != 2)
    {
        // 提示用户正确的命令行参数输入方式
        printf("please intput: rosrun vins vins_node [config file] \n"
               "for example: rosrun vins vins_node "
               "~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
        return 1;
    }

    // 读取配置文件路径
    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);

    // 读取参数设定
    readParameters(config_file);
    estimator.setParameter();

    // 检查是否禁用Eigen并行化
#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif

    // 输出警告信息
    ROS_WARN("waiting for image and imu...");

    // 注册发布者
    registerPub(n);

    // 创建IMU数据订阅者
    ros::Subscriber sub_imu;
    if (USE_IMU)
    {
        // 使用n.subscribe()函数订阅名为IMU_TOPIC的主题，设置队列大小为2000，
        // 并指定imu_callback作为接收到消息时的回调函数，同时使用ros::TransportHints().tcpNoDelay()设置传输选项。
        sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    }
    // 创建特征数据订阅者
    ros::Subscriber sub_feature = n.subscribe("/feature_tracker/feature", 2000, feature_callback);
    // 创建图像数据订阅者
    ros::Subscriber sub_img0 = n.subscribe(IMAGE0_TOPIC, 100, img0_callback);
    ros::Subscriber sub_img1;
    if (STEREO)
    {
        sub_img1 = n.subscribe(IMAGE1_TOPIC, 100, img1_callback);
    }
    // 创建重启信号订阅者
    ros::Subscriber sub_restart = n.subscribe("/vins_restart", 100, restart_callback);
    // 创建IMU开关信号订阅者
    ros::Subscriber sub_imu_switch = n.subscribe("/vins_imu_switch", 100, imu_switch_callback);
    // 创建相机开关信号订阅者
    ros::Subscriber sub_cam_switch = n.subscribe("/vins_cam_switch", 100, cam_switch_callback);

    // 创建线程用于同步处理
    std::thread sync_thread{sync_process};
    // 进入ROS事件循环
    ros::spin();

    return 0;
}
