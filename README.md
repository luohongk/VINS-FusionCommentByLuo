# VINS-FusionCommentByLuo

这个是仓库由罗宏昆同学注释的VINS-Fusion。特别感谢[秦通开源的VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)。

### 本项目的目的

一方面记录自己的学习过程，另外给准备学习VINS-Fusion这套优秀的代码的同学提供一个参考资料

### 项目结构

```
├── learnnote
│   ├── image
│   ├── processMeasurements测量线程分析.md
│   ├── sync_process线程分析.md
│   └── 入口函数.md
├── loop_fusion
│   ├── cmake
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── src
├── note
│   ├── 思维导图源文件
│   └── 思维到图图片
├── README.md
├── support_files
│   ├── brief_k10L6.bin
│   ├── brief_pattern.yml
│   ├── image
│   ├── paper
│   └── paper_bib.txt
├── vins_estimator
│   ├── cmake
│   ├── CMakeLists.txt
│   ├── launch
│   ├── package.xml
│   └── src
└── 问题.md
```

在相机模型，配置文件，全局融合模块基本没有动。learnnote主要是一些笔记，note是思维导图，vins_estimator节点中做了大量的注释可以进入看看，这也是VINS-Fusion的核心部分


### 简单运行

```
    roslaunch vins vins_rviz.launch
    rosrun vins vins_node ~/catkin_ws/src/VINS-FusionCommentByLuo/config/euroc/euroc_mono_imu_config.yaml 
    (optional) rosrun loop_fusion loop_fusion_node ~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_mono_imu_config.yaml 
    rosrun loop_fusion loop_fusion_node ~/catkin_ws/src/VINS-FusionCommentByLuo/config/euroc/euroc_mono_imu_config.yaml 
    rosbag play ~/data/EuRoC/MH_01_easy.bag
    rosbag play YOUR_DATASET_FOLDER/MH_01_easy.bag
```

运行loop_fusion

```
roslaunch vins vins_rviz.launch
rosrun loop_fusion loop_fusion_node ~/catkin_ws/src/VINS-FusionCommentByLuo/config/euroc/euroc_mono_imu_config.yaml 
rosbag play ~/data/EuRoC/MH_01_easy.bag
```
