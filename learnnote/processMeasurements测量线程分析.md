## processMeasurements测量线程分析

首先获取特征数据与IMU数据，特征的数据是从特征缓冲区获取，这个是前端处理的结果，IMU数据是从rosbag里面读取的。

这里只需要知道，我可以知道当前帧下我们可以获取前一帧到当前帧这个时间间隔的数据，然而IMM的采样频率要大于图像的采样频率。因此呢，在t0-t1两帧的时间戳下，是有很多IMU的数据的。每一组数据呢又包含六个元素，三维空间中加速度的三个分量与角速度的三个分量。在获取IMU数据的函数处理如下

```C++
bool Estimator::getIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accVector,
                               vector<pair<double, Eigen::Vector3d>> &gyrVector)
{
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
```

在获取到IMU数据后，如果没有初始化，用IMU数据初始化第一帧的位姿。如果已经初始哈化了，就直接进行IMU的预积分。预积分函数如下

```C++
processIMU(accVector[i].first, dt, accVector[i].second, gyrVector[i].second);
```

对于这个预积分是怎么个预积分法呢？？
