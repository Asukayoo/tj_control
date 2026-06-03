add_executable(test_movJ test_movJ.cpp)
target_link_libraries(test_movJ PRIVATE mv_control)
target_compile_definitions(test_movJ PRIVATE
  CMPL_LIN
  EIGEN_MAX_STATIC_ALIGN_BYTES=0
  MV_CONTROL_SIM
)
target_include_directories(test_movJ PRIVATE ${CMAKE_SOURCE_DIR}/mv_control/src)

add_executable(demo_ik demo_ik.cpp)
target_link_libraries(demo_ik PRIVATE mv_control)
target_compile_definitions(demo_ik PRIVATE
  CMPL_LIN
  EIGEN_MAX_STATIC_ALIGN_BYTES=0
)
target_include_directories(demo_ik PRIVATE ${CMAKE_SOURCE_DIR}/mv_control/src)
