cmake_minimum_required(VERSION 3.15)

set(TJ_TEST_MV_INCLUDE ${CMAKE_SOURCE_DIR}/mv_control/src)
set(TJ_TEST_TOOL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/tool)

# setcap 后须用 RPATH（非 RUNPATH）解析动态库与 pluginlib dlopen 依赖
function(tj_test_configure_runtime target)
  set_target_properties(${target} PROPERTIES
    BUILD_RPATH "${TJ_RUNTIME_LIB_DIRS}"
    INSTALL_RPATH "${TJ_RUNTIME_LIB_DIRS}"
  )
  target_link_options(${target} PRIVATE "LINKER:--disable-new-dtags")
endfunction()

add_library(tj_test_support STATIC
  tool/recorder.cpp
  tool/run_session.cpp
  tool/rt_thread.cpp
  tool/test_diag.cpp
)
target_link_libraries(tj_test_support PUBLIC mv_control)
target_include_directories(tj_test_support PUBLIC ${TJ_TEST_TOOL_DIR})
target_include_directories(tj_test_support PRIVATE ${TJ_TEST_MV_INCLUDE})

add_executable(test_enable test_enable.cpp)
target_link_libraries(test_enable PRIVATE tj_test_support)
target_compile_definitions(test_enable PRIVATE
    TJ_DATA_DEFAULT="${CMAKE_SOURCE_DIR}/data/test_enable")
tj_test_configure_runtime(test_enable)

add_executable(test_movj test_movj.cpp)
target_link_libraries(test_movj PRIVATE tj_test_support)
target_compile_definitions(test_movj PRIVATE
    TJ_DATA_DEFAULT="${CMAKE_SOURCE_DIR}/data/test_movj")
tj_test_configure_runtime(test_movj)

add_executable(test_servo test_servo.cpp)
target_link_libraries(test_servo PRIVATE tj_test_support)
target_compile_definitions(test_servo PRIVATE
    TJ_DATA_DEFAULT="${CMAKE_SOURCE_DIR}/data/test_ServoPByPico"
    TJ_PICO_CSV_DEFAULT="${CMAKE_SOURCE_DIR}/data/test_teleop_data/pico_record_20260615_211537.csv")
tj_test_configure_runtime(test_servo)

add_executable(test_rt_teleop test_rt_teleop.cpp)
target_link_libraries(test_rt_teleop PRIVATE tj_test_support)
target_include_directories(test_rt_teleop PRIVATE ${TJ_TEST_MV_INCLUDE})
target_compile_definitions(test_rt_teleop PRIVATE
    TJ_DATA_DEFAULT="${CMAKE_SOURCE_DIR}/data/test_rt_teleop")
tj_test_configure_runtime(test_rt_teleop)

# 编译后提示：默认需要 SCHED_FIFO，可运行 grant_rt_caps.sh
add_custom_target(tj_test_grant_rt_caps
  COMMAND ${CMAKE_COMMAND} -E echo
          "tj_test RT: bash ${CMAKE_SOURCE_DIR}/tj_test/grant_rt_caps.sh ${CMAKE_BINARY_DIR}"
  VERBATIM)
add_dependencies(tj_test_grant_rt_caps test_movj test_enable test_servo test_rt_teleop)
