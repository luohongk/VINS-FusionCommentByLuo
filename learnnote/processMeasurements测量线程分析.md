## processMeasurements测量线程分析

首先获取特征数据与IMU数据，特征的数据是从特征缓冲区获取，这个是前端处理的结果，IMU数据是从rosbag里面读取的。

这里只需要知道，我可以知道当前帧下我们可以获取前一帧到当前帧这个时间间隔的数据，然而IMU的采样频率要大于图像的采样频率。因此呢，在t0-t1两帧的时间戳下，是有很多IMU的数据的。每一组数据呢又包含六个元素，三维空间中加速度的三个分量与角速度的三个分量。在获取IMU数据的函数处理如下

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

在获取到IMU数据后，如果没有初始化，用IMU数据初始化第一帧的位姿。如果已经初始化了，就直接进行IMU的预积分。预积分函数如下

```C++
processIMU(accVector[i].first, dt, accVector[i].second, gyrVector[i].second);
```

在预积分函数之前，有一个关于dt的处理，这个处理还点可以说的，因为这个dt的计算分成了三块。前面通过getIMUInterval()获得了前一帧时间到当前帧时间的角速度，加速度。注意，这里获得的角速度的第一个值一定是上一帧的那个时刻的值吗？答案是不一定，因为这个角速度的时间戳并不一定与图像帧是对齐的，因此角速度的第一个值有时间有可能在prevTime之后，所以作者这里单独计算了一下prevTime与accVector**[**i**]**.first 之间的时间差，最后一个值同理，中间一块卡ui的dt其实就是imu的采样间隔，作者用的 dt = accVector**[**i**]**.first - accVector**[**i - 1**]**.first。这样操作下来，就能够保证所有的dt值的和一定是当前帧与上一帧的时间差！（妙啊！）

```C++
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
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
        // if(solver_flag != NON_LINEAR)
        tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

        // 保存时间戳列表
        dt_buf[frame_count].push_back(dt);

        // 保存加速度与角加速度列表
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);

        int j = frame_count;

        //未校准的加速度与角速度
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
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
```

对于这个积分是怎么个预积分法呢？？可以看到预积分传入的参数是一个时间戳，一个dt(也就是需要计算积分的时间间隔)，两个已经存了IMU数据的容器。具体的预积分的函数倒是没有几行代码。首先对first_imu进行判断，如果是第一次接受到imu数据，就初始化一下这个 acc_0， gyr_0。这里主要说说不是第一次接受到imu数据如何计算的，首先一些数据的保存就不用说了,比较关键的地方在于如下几行代码

```C++
        int j = frame_count;

        //未校准的加速度与角速度
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];

        // 似乎是根据未校准的角速度和时间增量 dt 来进行旋转矩阵的更新操作。
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

        // 对位置进行更新
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
        Vs[j] += dt * un_acc;
```



