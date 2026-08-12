cmake_minimum_required(VERSION 3.16)
project(ethercat_joint_control VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()
add_compile_options(-Wall -Wextra -Wno-unused-parameter)

# ── IgH EtherCAT Master ──────────────────────────────────────────────────
# 现场安装前缀是 /usr/local（不是厂商 README 写的 /usr/local/etherlab）
set(ETHERLAB_DIR "/usr/local" CACHE PATH "IgH EtherCAT Master 安装前缀")
find_path(ECRT_INCLUDE_DIR ecrt.h HINTS ${ETHERLAB_DIR}/include)
find_library(ECRT_LIBRARY NAMES ethercat HINTS ${ETHERLAB_DIR}/lib)
if(NOT ECRT_INCLUDE_DIR OR NOT ECRT_LIBRARY)
  message(FATAL_ERROR
    "找不到 IgH EtherCAT Master。\n"
    "  查找路径: ${ETHERLAB_DIR}/include/ecrt.h 与 ${ETHERLAB_DIR}/lib/libethercat.so\n"
    "  若安装在别处，请用 -DETHERLAB_DIR=<前缀> 指定。")
endif()
message(STATUS "IgH EtherCAT: ${ECRT_LIBRARY}")

# ── yaml-cpp ─────────────────────────────────────────────────────────────
find_package(yaml-cpp QUIET)
if(NOT yaml-cpp_FOUND)
  find_library(YAMLCPP_LIB NAMES yaml-cpp REQUIRED)
  find_path(YAMLCPP_INC yaml-cpp/yaml.h REQUIRED)
endif()

# ── HDF5 ─────────────────────────────────────────────────────────────────
find_package(HDF5 REQUIRED COMPONENTS C)
message(STATUS "HDF5: ${HDF5_LIBRARIES}")

# ── 核心库（可被 backend 与 tests 共用）───────────────────────────────────
add_library(ecjc_core STATIC
  backend/src/cia402/cia402.cpp
  backend/src/ethercat/scaling.cpp
  backend/src/trajectory/trajectory.cpp
  backend/src/controllers/controller.cpp
  backend/src/parameter/parameter.cpp
  backend/src/config.cpp
  backend/src/mock/mock_bus.cpp
  backend/src/realtime/realtime_task.cpp
  backend/src/communication/ipc_server.cpp
  backend/src/logger/data_logger.cpp
)
target_include_directories(ecjc_core PUBLIC backend/include)
if(yaml-cpp_FOUND)
  target_link_libraries(ecjc_core PUBLIC yaml-cpp)
else()
  target_include_directories(ecjc_core PUBLIC ${YAMLCPP_INC})
  target_link_libraries(ecjc_core PUBLIC ${YAMLCPP_LIB})
endif()
target_include_directories(ecjc_core PUBLIC ${HDF5_INCLUDE_DIRS} /usr/local/include)
target_link_libraries(ecjc_core PUBLIC ${HDF5_LIBRARIES} pthread)

# ── Git commit（metadata 落地用，Task 13；评审 Critical 2 后改为构建时刷新）──
# 原先用 execute_process() 在 CMake configure 阶段跑一次 git rev-parse，烤进
# target_compile_definitions——但日常"改代码 → commit → cmake --build"不会
# 触发 reconfigure，二进制里嵌的会是上一次 configure 时的陈旧 hash，比不记录
# 更危险（数据看起来绑了版本，实际绑错了）。
#
# 改为：add_custom_target(... ALL ...) 驱动 cmake/GenerateGitVersion.cmake，
# 让它在**每次 build**都重新跑一遍 git rev-parse 并写 git_version.h；脚本内部
# 做了"内容不变就不落盘"的比较，避免 hash 没变时也把 main.cpp 的编译判定为
# 过期（见脚本内注释）。main.cpp 用 __has_include 兜底 —— 找不到这个生成头时
# （例如绕开 CMake 的编译路径）自己把 ECJC_GIT_COMMIT 定义成 "unknown"。
set(ECJC_GIT_VERSION_HEADER "${CMAKE_BINARY_DIR}/generated/ecjc/git_version.h")
add_custom_target(ecjc_git_version ALL
  COMMAND ${CMAKE_COMMAND}
          -DECJC_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
          -DECJC_DST=${ECJC_GIT_VERSION_HEADER}
          -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/GenerateGitVersion.cmake
  BYPRODUCTS ${ECJC_GIT_VERSION_HEADER}
  COMMENT "刷新 git_version.h（每次 build 都重算，不只是 configure 时）"
  VERBATIM)

# ── Backend 可执行 ───────────────────────────────────────────────────────
add_executable(ecjc-backend
  backend/main.cpp
  backend/src/ethercat/igh_bus.cpp
)
target_include_directories(ecjc-backend PRIVATE ${ECRT_INCLUDE_DIR} ${HDF5_INCLUDE_DIRS}
                                                 "${CMAKE_BINARY_DIR}/generated")
target_link_libraries(ecjc-backend PRIVATE ecjc_core ${ECRT_LIBRARY} ${HDF5_LIBRARIES} pthread rt)
add_dependencies(ecjc-backend ecjc_git_version)

# ── 单元测试 ─────────────────────────────────────────────────────────────
enable_testing()
add_subdirectory(tests)

install(TARGETS ecjc-backend RUNTIME DESTINATION bin)
