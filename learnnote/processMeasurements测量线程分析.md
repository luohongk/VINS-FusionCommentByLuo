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

对于这个积分是怎么个预积分法呢？？可以看到预积分传入的参数是一个时间戳，一个dt(也就是需要计算积分的时间间隔)，两个已经存了IMU数据的容器。具体的预积分的函数倒是没有几行代码。首先对first_imu进行判断，如果是第一次接受到imu数据，就初始化一下这个 acc_0， gyr_0。这里主要说说非第一次接受到imu数据如何计算的，首先一些数据的保存就不用说了,比较关键的地方在于如下几行代码就是预积分的核心（篇幅原因，可以自己慢慢啃一肯预积分)

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

预积分结束，接下来进行图像处理模块，预积分最后可以得到每一帧初始位姿。在processimage模块中，传进去的就是图像的时间戳与图像的特征。首先进行第一步**addFeatureCheckParallax**的处理。传入这是第几帧，这个帧的图像特征，这个td是指当前帧与上一帧的时间差。

```C++
bool FeatureManager::addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, double td)
{
    // 输出输入特征数量
    ROS_DEBUG("input feature: %d", (int)image.size());
    // 输出当前特征数量
    ROS_DEBUG("num of feature: %d", getFeatureCount());

    // 初始化视差和特征数

    // 总视差
    double parallax_sum = 0;

    // 次新帧与次次新帧中的共同特征的数量
    int parallax_num = 0;

    // 被跟踪点的个数，非新特征点的个数
    last_track_num = 0;
    last_average_parallax = 0;
    new_feature_num = 0;
    long_track_num = 0;

    // 遍历传入的图像特征
    for (auto &id_pts : image)
    {
        // 创建当前帧的特征对象
        FeaturePerFrame f_per_fra(id_pts.second[0].second, td);
        // 判断是不是单目数据

        // assert语句用于进行断言检查，以确保条件为真。如果断言条件为假，程序将终止并生成一个错误消息。
        // 一般来说都是真哈，这里就相当于上了一个保险
        assert(id_pts.second[0].first == 0);

        // 如果特征点数量为2，添加右侧观测。因为开始构造特征帧的时候就是先构造左目，后构造右目
        // 构造的时候都把他们俩放到了特征容器里面，具体就去看trackImage()
        if (id_pts.second.size() == 2)
        {
            // 如果是双目的话，就把右目的数据放入到f_per_fra这个对象之中
            f_per_fra.rightObservation(id_pts.second[1].second);

            // assert语句用于进行断言检查，以确保条件为真。如果断言条件为假，程序将终止并生成一个错误消息。
            // 一般来说都是真哈，这里就相当于上了一个保险
            assert(id_pts.second[1].first == 1);
        }

        // 获取特征id
        // id_pts.first就是对于每一帧而言的特征点的ID
        // 与其说是特征点的ID,不如说是地图点的ID,因为其实左目右目的特征点对应的是同一个地图点，那么这个特征ID是一样的
        int feature_id = id_pts.first;

        // 查找特征id在特征数组中的位置，每一个地图点都分配一个全局ID
        // 这里的这个feature相当于是维护的一个feature的数据库
        // 每次新来一帧都要去这个数据库里面找之前的特征，如果没有，就添加一个新的特征
        auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId &it)
                          { return it.feature_id == feature_id; });

        // 如果遍历结束了，都没找到这个feature,那么就把
        if (it == feature.end())
        {
            // 把当前特征放入feature数据库中，方便下一帧来了在进行查找（其实也就相当于新加了一个地图点）
            feature.push_back(FeaturePerId(feature_id, frame_count));

            // 这里如果有右目的话说就把右边目数据放入feature最后一个容器对象的feature_per_frame中？？
            feature.back().feature_per_frame.push_back(f_per_fra);
            new_feature_num++;
        }
        // 如果特征id存在，说明左目已经放了数据了，则将前面构造好的特征对象添加至特征数组对应的特征id处
        // 这样每次如果新加入了一个特征，就在指定ID中加入一个，方便与计算这个ID对应的地图点是由多少个帧共同看到的。
        else if (it->feature_id == feature_id)
        {
            // 直接把右目数据放进入
            it->feature_per_frame.push_back(f_per_fra);

            // 这个表示当前特征如果有四个帧都看到了这个特征所对应的地图点，那就在last_track_num自增1
            // 这里基本能够表示当前帧与历史帧的相似度，如果last_track_num越大，那肯定相似性越高
            last_track_num++;
            // 计算长时间跟踪的特征数量
            if (it->feature_per_frame.size() >= 4)
                long_track_num++;
        }
    }

    // 如果满足某些条件，则返回true，这就表明这个帧属于关键帧了，那么会边缘化窗口内最旧的一帧
    // # mark 这个关键帧选取的条件有点迷
    if (frame_count < 2 || last_track_num < 20 || long_track_num < 40 || new_feature_num > 0.5 * last_track_num)
        return true;

    // 计算能被当前帧和其前两帧共同看到的特征点视差
    for (auto &it_per_id : feature)
    {
        // 判断特征是否在帧范围内并计算视差
        if (it_per_id.start_frame <= frame_count - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1)
        {
            parallax_sum += compensatedParallax2(it_per_id, frame_count);
            parallax_num++;
        }
    }

    // 若视差数为0，则返回true
    // 如果视差为0，说明当前帧和其前两帧压根就没有共同观测到的地图点。直接就是关键帧
    if (parallax_num == 0)
    {
        return true;
    }

    // 如果有共同观测到的地图点，那么就看看这个视差大不大，如果视差还比较大的话，那么就
    // 也当作关键帧，边缘化最旧的一帧
    else
    {
        ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        ROS_DEBUG("current parallax: %lf", parallax_sum / parallax_num * FOCAL_LENGTH);
        last_average_parallax = parallax_sum / parallax_num * FOCAL_LENGTH;
        return parallax_sum / parallax_num >= MIN_PARALLAX;
    }
}
```

这个函数addFeatureCheckParallax主要就是有关关键帧的选取与边缘化这个边缘化的策略是这样的，主要就是计算视差，如果当前帧与前两帧压根没有公共地图点（视差为0)，或者视差大于一个阈值，表示当前帧与前两帧的图像差距非常大，因此需要边缘化最后旧一帧（也就是关键帧，因为此时的新帧要设置成关键帧了)。否则就边缘化倒数第二旧的一帧。完了之后就进行相机校准。
