# VINS-FusionCommentByLuo

这个是仓库由罗宏昆同学注释的VINS-Fusion

```
    roslaunch vins vins_rviz.launch
    rosrun vins vins_node ~/catkin_ws/src/VINS-FusionCommentByLuo/config/euroc/euroc_mono_imu_config.yaml 
    (optional) rosrun loop_fusion loop_fusion_node ~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_mono_imu_config.yaml 
    rosbag play ~/data/EuRoC/MH_01_easy.bag
    rosbag play YOUR_DATASET_FOLDER/MH_01_easy.bag
```
