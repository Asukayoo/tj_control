cmake_minimum_required(VERSION 3.15)

set(MV_CONTROL_SDK_ROOT "${CMAKE_SOURCE_DIR}/TJ_SDK/TJ_FX_ROBOT_CONTRL_SDK" CACHE PATH
    "Marvin SDK root")

add_library(mv_control STATIC
  src/mv_control.cpp
  src/robot.cpp
  src/config.cpp
  src/hw_interface.cpp
  src/internal/ik.cpp
  src/internal/motion.cpp
  src/internal/sdk_map.cpp
  src/internal/diag.cpp
)

target_include_directories(mv_control
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${MV_CONTROL_SDK_ROOT}/contrlSDK100343
)

target_link_libraries(mv_control
  PUBLIC
    ruckig_part
)

find_library(MARVIN_SDK MarvinSDK
  PATHS
    ${MV_CONTROL_SDK_ROOT}/contrlSDK100343
    ${MV_CONTROL_SDK_ROOT}/DEMO_C++
  NO_DEFAULT_PATH
)

# KDL + kdl_parser（URDF 运动学）
list(APPEND CMAKE_PREFIX_PATH "/opt/ros/jazzy")
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)

# setcap 后 LD_LIBRARY_PATH 无效；须把 URDF/plugin 链作为直接依赖预载入
set(_MV_ROS_RUNTIME_LIBS
  urdf
  urdf_xml_parser
  urdfdom_model
  urdfdom_world
  urdfdom_sensor
  class_loader
  console_bridge
  tinyxml2
  ament_index_cpp
  rcpputils
  rcutils
)
set(MV_ROS_RUNTIME_LIBS "")
foreach(_lib IN LISTS _MV_ROS_RUNTIME_LIBS)
  find_library(_MV_ROS_LIB_${_lib} ${_lib} PATHS ${TJ_RUNTIME_LIB_DIRS} NO_DEFAULT_PATH)
  if(_MV_ROS_LIB_${_lib})
    list(APPEND MV_ROS_RUNTIME_LIBS ${_MV_ROS_LIB_${_lib}})
  else()
    message(FATAL_ERROR "mv_control: required runtime lib ${_lib} not found in ${TJ_RUNTIME_LIB_DIRS}")
  endif()
endforeach()

message(STATUS "mv_control: ROS runtime libs linked: ${MV_ROS_RUNTIME_LIBS}")

target_link_libraries(mv_control PUBLIC
  orocos-kdl
  kdl_parser::kdl_parser
  ${MV_ROS_RUNTIME_LIBS}
)

if(MARVIN_SDK)
  target_link_libraries(mv_control PRIVATE ${MARVIN_SDK})
else()
  message(FATAL_ERROR "MarvinSDK not found; install control SDK")
endif()

find_package(yaml-cpp REQUIRED)
target_link_libraries(mv_control PUBLIC yaml-cpp::yaml-cpp)

find_package(Eigen3 3.3 QUIET)
if(Eigen3_FOUND)
  target_link_libraries(mv_control PUBLIC Eigen3::Eigen)
else()
  target_include_directories(mv_control PUBLIC /usr/include/eigen3)
endif()

target_compile_definitions(mv_control PUBLIC
    MV_CONTROL_CONFIG_DEFAULT="${CMAKE_SOURCE_DIR}/mv_control/config/config.yaml")
# Must be PUBLIC: headers expose Eigen-fixed types (V6d/ImpConfig/MvConfig). PRIVATE-only
# caused ODR (test TU sizeof MvConfig=1120 vs lib=1112) → 8-byte heap overrun in ctor.
target_compile_definitions(mv_control PUBLIC EIGEN_MAX_STATIC_ALIGN_BYTES=0)
target_compile_definitions(mv_control PRIVATE CMPL_LIN)
message(STATUS "mv_control: runtime SIM/HW via Init(is_sim)")
