# 设置要求的 CMake 的最小版本
cmake_minimum_required(VERSION 2.8.3)
# 设置项目名称
project(vins)

# 设置构建类型为 Release
set(CMAKE_BUILD_TYPE "Release")
# 设置 C++ 编译标准为 c++11
set(CMAKE_CXX_FLAGS "-std=c++11")
# 设置 Release 模式下的编译标志
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# 查找需要的 ROS package
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    cv_bridge
    camera_models
    image_transport)

find_package(OpenCV REQUIRED)

# 查找 Ceres 库
find_package(Ceres REQUIRED PATHS "/home/lhk/Thirdparty/Ceres")

# 包含相关的头文件路径
include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

# 设置 CMake 模块路径并查找 Eigen3 库
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# 设置 Catkin package
catkin_package()

# 添加 vins_lib 库及其源文件
add_library(vins_lib
    src/estimator/parameters.cpp
    src/estimator/estimator.cpp
    src/estimator/feature_manager.cpp
    src/factor/pose_local_parameterization.cpp
    src/factor/projectionTwoFrameOneCamFactor.cpp
    src/factor/projectionTwoFrameTwoCamFactor.cpp
    src/factor/projectionOneFrameTwoCamFactor.cpp
    src/factor/marginalization_factor.cpp
    src/utility/utility.cpp
    src/utility/visualization.cpp
    src/utility/CameraPoseVisualization.cpp
    src/initial/solve_5pts.cpp
    src/initial/initial_aligment.cpp
    src/initial/initial_sfm.cpp
    src/initial/initial_ex_rotation.cpp
    src/featureTracker/feature_tracker.cpp)
# 链接相关库
target_link_libraries(vins_lib ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})

# 添加 vins_node 及其源文件
add_executable(vins_node src/rosNodeTest.cpp)
# 链接 vins_lib 库
target_link_libraries(vins_node vins_lib) 

# 添加 kitti_odom_test 及其源文件
add_executable(kitti_odom_test src/KITTIOdomTest.cpp)
# 链接 vins_lib 库
target_link_libraries(kitti_odom_test vins_lib) 

# 添加 kitti_gps_test 及其源文件
add_executable(kitti_gps_test src/KITTIGPSTest.cpp)
# 链接 vins_lib 库
target_link_libraries(kitti_gps_test vins_lib)