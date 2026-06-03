cmake_minimum_required(VERSION 3.15)

# SIM 模式：默认开启，不连接真实硬件，反馈 resp 跟随 ref
option(MV_CONTROL_SIM "Simulate hardware; resp tracks ref without Marvin control link" ON)

set(MV_CONTROL_SDK_ROOT "${CMAKE_SOURCE_DIR}/TJ_SDK/TJ_FX_ROBOT_CONTRL_SDK" CACHE PATH
    "Marvin SDK root")

add_library(mv_control STATIC
  src/mv_control.cpp
  src/robot.cpp
  src/config.cpp
  src/internal/ik.cpp
  src/internal/motion.cpp
)

target_include_directories(mv_control
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${MV_CONTROL_SDK_ROOT}/contrlSDK
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${MV_CONTROL_SDK_ROOT}/kinematicsSDK
)

target_link_libraries(mv_control
  PUBLIC
    ruckig_part
)

find_library(MARVIN_SDK MarvinSDK
  PATHS
    ${MV_CONTROL_SDK_ROOT}/contrlSDK
    ${MV_CONTROL_SDK_ROOT}/DEMO_C++
  NO_DEFAULT_PATH
)
find_library(KINE_SDK Kine
  PATHS
    ${MV_CONTROL_SDK_ROOT}/kinematicsSDK
    ${MV_CONTROL_SDK_ROOT}/DEMO_C++
  NO_DEFAULT_PATH
)

if(KINE_SDK)
  target_link_libraries(mv_control PRIVATE ${KINE_SDK})
else()
  message(WARNING "Kine SDK not found; mv_control FK/IK unavailable")
endif()

if(NOT MV_CONTROL_SIM)
  if(MARVIN_SDK)
    target_link_libraries(mv_control PRIVATE ${MARVIN_SDK})
  else()
    message(FATAL_ERROR "MarvinSDK not found; disable MV_CONTROL_SIM or install control SDK")
  endif()
endif()

find_package(yaml-cpp REQUIRED)
target_link_libraries(mv_control PUBLIC yaml-cpp::yaml-cpp)

find_package(Eigen3 3.3 QUIET)
if(Eigen3_FOUND)
  target_link_libraries(mv_control PUBLIC Eigen3::Eigen)
else()
  target_include_directories(mv_control PUBLIC /usr/include/eigen3)
endif()

target_compile_definitions(mv_control PRIVATE CMPL_LIN EIGEN_MAX_STATIC_ALIGN_BYTES=0)
if(MV_CONTROL_SIM)
  target_compile_definitions(mv_control PRIVATE MV_CONTROL_SIM)
  message(STATUS "mv_control: SIM mode (no hardware link)")
else()
  message(STATUS "mv_control: hardware mode")
endif()
