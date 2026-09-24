cmake_minimum_required(VERSION 3.16)

# tools 使用 std::thread / std::mutex：Ubuntu 20.04(glibc 2.31) 下 pthread 是与 libc 分离的库，
# 只链接 tools 的目标（如 split_video / calibrate_*）必须显式带上 pthread，否则报
# "DSO missing from command line"。这里用 Threads::Threads 统一传播。
find_package(Threads REQUIRED)

add_library(tools OBJECT 
    exiter.cpp
    extended_kalman_filter.cpp
    ransac_sine_fitter.cpp
    img_tools.cpp
    math_tools.cpp
    plotter.cpp
    trajectory.cpp
    recorder.cpp
    logger.cpp
    pid.cpp
    crc.cpp
)

target_link_libraries(tools PUBLIC Threads::Threads)