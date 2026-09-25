cmake_minimum_required(VERSION 3.16)

# set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake")

# find_package(OpenVINO REQUIRED COMPONENTS Runtime)

add_subdirectory(planner/tinympc)

add_library(auto_aim OBJECT 
    armor.cpp
    # classifier.cpp 
    # detector.cpp
    solver.cpp
    aimer.cpp
    target.cpp
    tracker.cpp
    voter.cpp
    shooter.cpp
    yolo.cpp                # 后端分发：TensorRT > ONNX Runtime > OpenCV DNN（AGENTS.md §4）
    yolos/yolov5_onnx.cpp   # OpenCV DNN 推理后端（兜底，需 OpenCV >= 4.9）
    # yolos/yolov5.cpp      # 原 OpenVINO 版，待 TensorRT 后端替换
    # yolos/yolov8.cpp
    # yolos/yolo11.cpp
    multithread/commandgener.cpp
    multithread/mt_detector.cpp  # 已去掉 ov::（工作线程+队列，对外接口不变）
    planner/planner.cpp
)

# ONNX Runtime 推理后端（x86_64 / WSL2 主力后端）：只有第三方库就位时才参与编译。
# 头文件与实现都由同一个宏 SPVISION_HAS_ORT 保护（tasks/auto_aim/yolos/yolov5_ort.hpp），
# yolo.cpp 也用它做条件包含，因此三者必须一致。
if(TARGET onnxruntime::onnxruntime)
    target_sources(auto_aim PRIVATE yolos/yolov5_ort.cpp)
    target_compile_definitions(auto_aim PUBLIC SPVISION_HAS_ORT)
    # 用 plain 签名：本文件下方既有 target_link_libraries(auto_aim ...) 是 plain，不能混用
    target_link_libraries(auto_aim onnxruntime::onnxruntime)
    message(STATUS "auto_aim: ONNX Runtime 后端已启用（SPVISION_HAS_ORT）")
else()
    message(STATUS "auto_aim: 无 ONNX Runtime，检测后端仅 OpenCV DNN")
endif()

target_link_libraries(auto_aim io openvino::runtime tinympcstatic)
