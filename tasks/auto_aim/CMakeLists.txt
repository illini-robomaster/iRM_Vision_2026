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
    yolo.cpp                # 后端分发（默认走 ONNX/OpenCV DNN，见 AGENTS.md §4）
    yolos/yolov5_onnx.cpp   # OpenCV DNN 推理后端（无需 OpenVINO / TensorRT）
    # yolos/yolov5.cpp      # 原 OpenVINO 版，待 TensorRT 后端替换
    # yolos/yolov8.cpp
    # yolos/yolo11.cpp
    multithread/commandgener.cpp
    multithread/mt_detector.cpp  # 已去掉 ov::（工作线程+队列，对外接口不变）
    planner/planner.cpp
)

target_link_libraries(auto_aim io openvino::runtime tinympcstatic)
