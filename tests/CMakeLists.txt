# 轻量自研测试框架：不引入 gtest 依赖，保持工程可离线构建。
add_library(ecjc_test_main STATIC test_main.cpp)
target_include_directories(ecjc_test_main PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ecjc_test_main PUBLIC ecjc_core)

# 用 glob 而不是硬编码列表：新增 test_*.cpp 丢进本目录即自动纳入，
# 避免"加了测试文件却静默不参与构建、ctest 依然全绿"这种覆盖率回退。
file(GLOB ECJC_TEST_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/test_*.cpp")
list(REMOVE_ITEM ECJC_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/test_main.cpp")

foreach(src ${ECJC_TEST_SOURCES})
  get_filename_component(t ${src} NAME_WE)
  add_executable(${t} ${src})
  target_link_libraries(${t} PRIVATE ecjc_test_main)
  add_test(NAME ${t} COMMAND ${t})
  set_tests_properties(${t} PROPERTIES TIMEOUT 120)
endforeach()
