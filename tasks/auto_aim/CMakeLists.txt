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
    yolo.cpp                # 后端分发（auto/onnx_dnn 走 ONNX，trt 走 TensorRT，见 AGENTS.md §4）
    yolos/yolov5_onnx.cpp   # OpenCV DNN 推理后端（无需 OpenVINO / TensorRT）
    yolos/yolov5_postprocess.cpp  # 各后端共享的 letterbox/parse（唯一实现，避免漂移）
    yolos/yolov5_trt.cpp    # TensorRT 推理后端（本机未装 TRT 时编译为占位实现，运行时报错）
    # yolos/yolov5.cpp      # 原 OpenVINO 版，待 TensorRT 后端替换
    # yolos/yolov8.cpp
    # yolos/yolo11.cpp
    multithread/commandgener.cpp
    multithread/mt_detector.cpp  # 已去掉 ov::（工作线程+队列，对外接口不变）
    planner/planner.cpp
)

target_link_libraries(auto_aim io openvino::runtime tinympcstatic)

# ---------------- TensorRT（可选，AGENTS.md §4）----------------
# 装了 TensorRT 才有 HAVE_TENSORRT=1，yolov5_trt 后端才是真实现；
# 没装（x86 CI / 笔记本）时该后端退化为「构造即抛清晰异常」，不影响其他目标编译。
find_path(NVINFER_INCLUDE_DIR NvInfer.h
    PATHS /usr/include/aarch64-linux-gnu /usr/include/x86_64-linux-gnu /usr/include)
find_path(CUDART_INCLUDE_DIR cuda_runtime_api.h
    PATHS /usr/local/cuda-11.4/include /usr/local/cuda-11.4/targets/aarch64-linux/include
          /usr/local/cuda/include /usr/local/cuda/targets/aarch64-linux/include /usr/include)
find_library(NVINFER_LIB nvinfer
    PATHS /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu /usr/lib)
find_library(CUDART_LIB cudart
    PATHS /usr/local/cuda-11.4/lib64 /usr/local/cuda-11.4/targets/aarch64-linux/lib
          /usr/local/cuda/lib64 /usr/lib/aarch64-linux-gnu)

if(NVINFER_INCLUDE_DIR AND CUDART_INCLUDE_DIR AND NVINFER_LIB AND CUDART_LIB)
    message(STATUS "TensorRT found: ${NVINFER_LIB} -> HAVE_TENSORRT=1")
    target_include_directories(auto_aim PUBLIC ${NVINFER_INCLUDE_DIR} ${CUDART_INCLUDE_DIR})
    target_link_libraries(auto_aim PUBLIC ${NVINFER_LIB} ${CUDART_LIB})
    target_compile_definitions(auto_aim PUBLIC HAVE_TENSORRT=1)
else()
    message(STATUS "TensorRT NOT found -> yolov5_trt 后端为占位实现（见 AGENTS.md §4.0）")
endif()
