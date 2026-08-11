# 轻量自研测试框架：不引入 gtest 依赖，保持工程可离线构建。
add_library(ecjc_test_main STATIC test_main.cpp)
target_include_directories(ecjc_test_main PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ecjc_test_main PUBLIC ecjc_core)

foreach(t test_cia402 test_scaling test_trajectory test_ring_buffer)
  add_executable(${t} ${t}.cpp)
  target_link_libraries(${t} PRIVATE ecjc_test_main)
  add_test(NAME ${t} COMMAND ${t})
endforeach()
