cmake_minimum_required(VERSION 3.0.2)
project(eskf)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

## 找到Eigen3库
find_package(Eigen3 REQUIRED)
## Declare catkin package
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a C++ executable
## 添加可执行目标，并链接依赖库
add_executable(relative_orientation src/simple_node.cpp src/RelativeOrientation.cpp)

## 目标需要链接哪些库
target_link_libraries(relative_orientation
  ${catkin_LIBRARIES}
  Eigen3::Eigen  # 链接Eigen库
)
