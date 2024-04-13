# 设置CMake最低版本要求为2.8.3
cmake_minimum_required(VERSION 2.8.3)
# 设置工程名称为camera_models
project(camera_models)

# 设置构建类型为Release
set(CMAKE_BUILD_TYPE "Release")
# 设置C++编译器标准为C++11
set(CMAKE_CXX_FLAGS "-std=c++11")
# Release模式下的编译选项，包含优化等级和位置无关代码标志
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fPIC")

# 查找catkin依赖的ROS包
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    )

# 查找Boost库
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
# 包含Boost库的头文件路径
include_directories(${Boost_INCLUDE_DIRS})

# 查找OpenCV库
find_package(OpenCV REQUIRED)

# 查找Ceres库
find_package(Ceres REQUIRED)
# 包含Ceres库的头文件路径
include_directories(${CERES_INCLUDE_DIRS})

# 设置catkin工程的信息
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES camera_models
    CATKIN_DEPENDS roscpp std_msgs
    )

# 包含catkin相关的头文件路径
include_directories(
    ${catkin_INCLUDE_DIRS}
    )

# 包含include目录下的头文件路径
include_directories("include")

# 定义可执行文件Calibrations及其源文件
add_executable(Calibrations 
    src/intrinsic_calib.cc
    src/chessboard/Chessboard.cc
    src/calib/CameraCalibration.cc
    src/camera_models/Camera.cc
    src/camera_models/CameraFactory.cc
    src/camera_models/CostFunctionFactory.cc
    src/camera_models/PinholeCamera.cc
    src/camera_models/PinholeFullCamera.cc
    src/camera_models/CataCamera.cc
    src/camera_models/EquidistantCamera.cc
    src/camera_models/ScaramuzzaCamera.cc
    src/sparse_graph/Transform.cc
    src/gpl/gpl.cc
    src/gpl/EigenQuaternionParameterization.cc)

# 定义静态库camera_models及其源文件
add_library(camera_models
    src/chessboard/Chessboard.cc
    src/calib/CameraCalibration.cc
    src/camera_models/Camera.cc
    src/camera_models/CameraFactory.cc
    src/camera_models/CostFunctionFactory.cc
    src/camera_models/PinholeCamera.cc
    src/camera_models/PinholeFullCamera.cc
    src/camera_models/CataCamera.cc
    src/camera_models/EquidistantCamera.cc
    src/camera_models/ScaramuzzaCamera.cc
    src/sparse_graph/Transform.cc
    src/gpl/gpl.cc
    src/gpl/EigenQuaternionParameterization.cc)

# 链接Calibrations可执行文件所需的库
target_link_libraries(Calibrations ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
# 链接camera_models静态库所需的库
target_link_libraries(camera_models ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
