cmake_minimum_required(VERSION 3.15)
project(tj_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(MV_CONTROL_SDK_ROOT "${CMAKE_SOURCE_DIR}/TJ_SDK/TJ_FX_ROBOT_CONTRL_SDK" CACHE PATH
    "Marvin SDK root")
set(TJ_ROS_LIB_DIR "/opt/ros/jazzy/lib" CACHE PATH "ROS Jazzy 运行时库目录")

# setcap 后 LD_LIBRARY_PATH 无效；RPATH 须覆盖 ROS 多架构子目录与系统库
set(TJ_RUNTIME_LIB_DIRS
  "${TJ_ROS_LIB_DIR}"
  "${TJ_ROS_LIB_DIR}/x86_64-linux-gnu"
  "/usr/lib/x86_64-linux-gnu"
  "${MV_CONTROL_SDK_ROOT}/contrlSDK100343"
)
list(APPEND CMAKE_BUILD_RPATH ${TJ_RUNTIME_LIB_DIRS})

add_subdirectory(third_part/ruckig_part)
add_subdirectory(mv_control)
add_subdirectory(tj_test)
