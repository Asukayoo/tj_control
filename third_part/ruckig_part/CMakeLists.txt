cmake_minimum_required(VERSION 3.15)

project(ruckig_part VERSION 0.17.3 LANGUAGES CXX)

# 离线位置轨迹子集（对应 examples/02_position_offline）
add_library(ruckig_part STATIC
  src/ruckig/brake.cpp
  src/ruckig/position_first_step1.cpp
  src/ruckig/position_first_step2.cpp
  src/ruckig/position_second_step1.cpp
  src/ruckig/position_second_step2.cpp
  src/ruckig/position_third_step1.cpp
  src/ruckig/position_third_step2.cpp
  src/ruckig/velocity_second_step1.cpp
  src/ruckig/velocity_second_step2.cpp
  src/ruckig/velocity_third_step1.cpp
  src/ruckig/velocity_third_step2.cpp
)

target_compile_features(ruckig_part PUBLIC cxx_std_20)
target_include_directories(ruckig_part PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(MSVC)
  target_compile_definitions(ruckig_part PUBLIC _USE_MATH_DEFINES)
endif()

add_library(ruckig_part::ruckig_part ALIAS ruckig_part)

option(RUCKIG_PART_BUILD_EXAMPLE "Build example/02_position_offline" ON)
if(RUCKIG_PART_BUILD_EXAMPLE)
  add_executable(example_position_offline example/02_position_offline.cpp)
  target_link_libraries(example_position_offline PRIVATE ruckig_part)
endif()

install(TARGETS ruckig_part
  EXPORT ruckig_part-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)
install(EXPORT ruckig_part-targets
  NAMESPACE ruckig_part::
  DESTINATION lib/cmake/ruckig_part
)
