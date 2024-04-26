## sync_process线程分析

总体来说这个线程就是用来对数据缓冲区的数据进行对齐处理，然后将对其后的数据推送到全局对象estimator中，主要是通过inputImage函数进行推送。

针对双目图像对齐，具体来说就是要保证左右目图像数据的时间戳的差小于0.003s。如果哪一个缓冲区的队列的头指针的时间戳晚了，就让头指针对应的数据出队，然后这时候缓冲区头指针对应的新的图像时间戳是往后推迟了（这里倒不是很关键，稍微了解一下作用)

```C++
 // 0.003秒的时间容差
                if (time0 < time1 - 0.003) // 如果图像1比图像2早超过了0.003秒
                {
                    img0_buf.pop();         // 丢弃图像1
                    printf("throw img0\n"); // 打印丢弃图像1的消息
                }
                else if (time0 > time1 + 0.003) // 如果图像1比图像2晚超过了0.003秒
                {
                    img1_buf.pop();         // 丢弃图像2
                    printf("throw img1\n"); // 打印丢弃图像2的消息
                }
```

接下来将图像的时间戳数据以及图像数据放入estimator对象，进入前端的数据处理。

## 前端入口函数解释

这里进入inputImage后进行前端的特征提取，执行trackImage函数，然后将特征保存到featureFrame。可以看到featureFrame是一个map类型的数据，前面是一个整型数据，后面还有一个pair的一对数据。

```C++
map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
```

这个数据结构的意思是：第一个int表示特征点的id信息，pair里面的int表示是单目还是双目里面的特征点信息，单目的话就是0，双目的话就是1，7纬度的向量比表示归一化平面的坐标（x,y,z,z=1）,像素坐标(u,v),速度(vx,vy)。

好了弄清楚前端的返回值了，接下来具体看一看前端的代码

## 前端特征提取

```C++
      // 如果具有预测的特征点
        if(hasPrediction)
        {
            // 使用光流法跟踪先前特征点到当前帧
            cur_pts = predict_pts;
            cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 1, 

            // 定义了迭代终止的条件，即迭代次数达到30或误差小于0.01时停止迭代。
            cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
        
            // 统计成功跟踪的特征点数量
            int succ_num = 0;
            for (size_t i = 0; i < status.size(); i++)
            {
                if (status[i])
                    succ_num++;
            }

            // 如果成功跟踪的特征点数量小于10，重新进行光流法跟踪
            if (succ_num < 10)
               // 不使用初始光流估计（不传入cv::OPTFLOW_USE_INITIAL_FLOW参数
               cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);
        }
        else
            cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);
```

对于上述代码可以看到，是先进行是否有预测的特征点。这里就有点门道了，这hasPrediction是在哪里设置的呢？？在processImage中的predictPtsInNextFrame();中设置，表示对当前帧的位姿估计完成了之后，对下一帧进行预测，只要是使用了多线程，就一定会对下一帧进行预测，因此多线程情况下，bool类型的hasPrediction就一直是true。有关这个预测函数，是怎么进行预测的呢？？

```C++
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
```

这个预测函数中，获取当前帧和前一帧的位姿矩阵。通过调用getPoseInWorldFrame()函数，分别获取当前帧的位姿curT和前一帧的位姿prevT。使用常速度模型进行位姿预测。通过将当前帧的位姿curT与前一帧的位姿prevT求逆并相乘，得到预测的下一帧位姿nextT。这样可以得到当前帧与下一帧之间的变换矩阵。然后获取当前特征点的深度通过相机投影把三维地图点投影到下一帧的相机坐标系。获得像素点在下一帧中的坐标，预测完成！

好了，有了预测那么就有了初始光流，有了初始光流就可以通过光流追踪追踪通过在下一帧图像中的位置。

当没有初始光流信息时，cv::calcOpticalFlowPyrLK()使用的是金字塔光流算法（pyramidal Lucas-Kanade算法）。该算法通过构建图像金字塔，在不同的尺度上计算光流，以处理图像中的尺度变化和大位移。

光流追踪完毕，我们现在知道了当前帧特征点的位置就可以反向光流追踪，所谓的反向光流就是把当前帧的特征点追踪上一帧的特征点的位置注意，这里是有初始光流的。

```C++
// 是否进行反向光流法
        if(FLOW_BACK)
        {
            // 创建一个存储反向光流跟踪状态的向量reverse_status
            vector<uchar> reverse_status;

            //反向光流跟踪的结果存储在reverse_pts中
            vector<cv::Point2f> reverse_pts = prev_pts;
            cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 1, 
            cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
        
            // 对于每个特征点，如果对应的正向光流跟踪状态status[i]为1，反向光流跟踪状态reverse_status[i]也为1，
            // 并且先前特征点与反向跟踪得到的特征点之间的距离小于等于0.5，那么将该特征点的状态更新为1
            for(size_t i = 0; i < status.size(); i++)
            {
                if(status[i] && reverse_status[i] && distance(prev_pts[i], reverse_pts[i]) <= 0.5)
                {
                    status[i] = 1;
                }
                else
                    status[i] = 0;
            }
        }
```

通过反向光流追踪，可以删除一些比较误差比较大的特征点，这里也算是一种优化策略吧。

光流正向反向处理完毕，就开始进行补点，这里所谓的补点是这样的，由于config文件中读取了一个最大点数，如果少于最大点就进行补充，补充的Harris角点。

接下来去畸变且参考上一帧计算特征点移动的速度。这个速度是用于返回特征帧的。

根据左目特征点，光流追踪右目特征点（如果有右目的话）这里是没有初始光流的。

```C++
// 如果右目图像不为空且为立体摄像头
    if(!_img1.empty() && stereo_cam)
    {
        // 清空右目图像的ID和点
        ids_right.clear();
        cur_right_pts.clear();
        cur_un_right_pts.clear();
        right_pts_velocity.clear();
        cur_un_right_pts_map.clear();
  
        // 如果当前点不为空
        // mark 这里为什么要做一个判断？？？
        // done 这里是根据左目的特征点去光流追踪右目的特征点。

        if(!cur_pts.empty())
        {
            // 计算光流并进行反向检查
            vector<cv::Point2f> reverseLeftPts;
            vector<uchar> status, statusRightLeft;
            vector<float> err;
            cv::calcOpticalFlowPyrLK(cur_img, rightImg, cur_pts, cur_right_pts, status, err, cv::Size(21, 21), 3);
            if(FLOW_BACK)
            {
                cv::calcOpticalFlowPyrLK(rightImg, cur_img, cur_right_pts, reverseLeftPts, statusRightLeft, err, cv::Size(21, 21), 3);
                for(size_t i = 0; i < status.size(); i++)
                {
                    if(status[i] && statusRightLeft[i] && inBorder(cur_right_pts[i]) && distance(cur_pts[i], reverseLeftPts[i]) <= 0.5)
                        status[i] = 1;
                    else
                        status[i] = 0;
                }
            }
  
            // 复制ID并根据状态缩减左图像和右图像的点
            // mark 这里的这个ids不是左目的吗？ 为什么把左目的ids赋值给右目？？？
            // done 由于是根据左目特征点光流追踪右目的特征点，因此左目右目ID是相同的。都对应的是同一个地图点
            ids_right = ids;
            reduceVector(cur_right_pts, status);
            reduceVector(ids_right, status);
  
            // 对右侧点进行畸变矫正并计算点速度
            cur_un_right_pts = undistortedPts(cur_right_pts, m_camera[1]);
            right_pts_velocity = ptsVelocity(ids_right, cur_un_right_pts, cur_un_right_pts_map, prev_un_right_pts_map);
        }
        prev_un_right_pts_map = cur_un_right_pts_map;
    }
```

最后就是构造特征帧，把该传入的数据都传入u到特征帧。
