# GenerateGitVersion.cmake —— 生成 git_version.h，构建时（不只是 configure 时）刷新。
#
# 背景（评审 Critical 2）：原来用 execute_process() 在 configure 阶段跑一次
# `git rev-parse --short HEAD`，烤进 target_compile_definitions。日常
# "改代码 → commit → cmake --build" 不会触发 CMake reconfigure，于是二进制里
# 嵌的还是上一次 configure 时的旧 hash——metadata 里的 git_commit 把数据绑到
# 错误的代码版本，比完全不记录还危险。
#
# 现改为：用 add_custom_target(... ALL ...)（在 CMakeLists.txt 里绑定）驱动本脚本
# 在**每次 build**都重新执行一遍 git rev-parse，而不是只在 configure 时跑一次。
# 但直接每次都 file(WRITE) 会导致 git_version.h 的 mtime 每次构建都变，进而让
# #include 它的 main.cpp 每次都被判定为过期、全量重编——即使 hash 根本没变。
# 所以这里先算出新内容，与磁盘上已有内容比较，只有真的不同才落盘（惯用的
# "compare-then-write" 模式，等价于 configure_file(... COPYONLY) 那种只在内容
# 变化时才更新 mtime 的行为，但 configure_file 本身只在 configure 阶段跑，
# 这里手动实现同样的比较逻辑以便能在 build 阶段复用）。
#
# 输入（由调用方经 -D 传入）：
#   ECJC_SOURCE_DIR —— git 仓库根目录（找不到 .git 或 git 不存在时优雅降级为 "unknown"）
#   ECJC_DST        —— 要写出的头文件路径

set(_ecjc_commit "unknown")

find_package(Git QUIET)
if(GIT_EXECUTABLE)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
    WORKING_DIRECTORY ${ECJC_SOURCE_DIR}
    OUTPUT_VARIABLE _ecjc_commit_out
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
    RESULT_VARIABLE _ecjc_git_rc)
  if(_ecjc_git_rc EQUAL 0 AND NOT _ecjc_commit_out STREQUAL "")
    set(_ecjc_commit "${_ecjc_commit_out}")
  endif()
endif()

set(_ecjc_content
"// 自动生成，见 cmake/GenerateGitVersion.cmake —— 每次 build 都会重新计算 git hash，
// 不是只在 CMake configure 时算一次（那样日常 build 会嵌入陈旧的旧 hash）。
// 不在 git 仓库里构建（例如从 tarball 构建）时优雅降级为 \"unknown\"。
#pragma once
#define ECJC_GIT_COMMIT \"${_ecjc_commit}\"
")

set(_ecjc_existing "")
if(EXISTS "${ECJC_DST}")
  file(READ "${ECJC_DST}" _ecjc_existing)
endif()

if(NOT _ecjc_existing STREQUAL _ecjc_content)
  get_filename_component(_ecjc_dst_dir "${ECJC_DST}" DIRECTORY)
  file(MAKE_DIRECTORY "${_ecjc_dst_dir}")
  file(WRITE "${ECJC_DST}" "${_ecjc_content}")
  message(STATUS "git_version.h 已更新: ECJC_GIT_COMMIT=${_ecjc_commit}")
endif()
