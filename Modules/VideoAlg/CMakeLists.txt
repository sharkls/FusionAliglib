project(VideoAlg)

message("CMAKE_SOURCE_DIR= " ${CMAKE_SOURCE_DIR})
message("VideoAlg CMAKE_PLATFORM " ${CMAKE_PLATFORM})
add_compile_options(-fvisibility=hidden)
add_compile_options(-fPIC)
add_compile_options(-shared)
add_compile_options(-fext-numeric-literals)

include_directories(${CMAKE_SOURCE_DIR}/FastddsSer/data)
include_directories(${CMAKE_SOURCE_DIR}/FastddsSer/param)
include_directories(${CMAKE_SOURCE_DIR}/Include/AlgParam)
include_directories(${CMAKE_SOURCE_DIR}/Include/Common)
include_directories(${CMAKE_SOURCE_DIR}/Include/Interface)
include_directories(${CMAKE_SOURCE_DIR}/Include/Thirdpart)
include_directories(${CMAKE_SOURCE_DIR}/Include/Thirdpart/av_opencv)
include_directories(${CMAKE_SOURCE_DIR}/Include/Thirdpart/cpp_algorithm)
include_directories(${CMAKE_SOURCE_DIR}/Include/Thirdpart/eigen3)
include_directories(${CMAKE_SOURCE_DIR}/Include/Thirdpart/Log)
include_directories(${CMAKE_SOURCE_DIR}/Include/Thirdpart/Log/glog)
include_directories(${CMAKE_SOURCE_DIR}/Include/Thirdpart/fastdds)
include_directories(${CMAKE_SOURCE_DIR}/Include/Thirdpart/yaml-cpp)
include_directories(${CMAKE_SOURCE_DIR}/Modules/Common)
include_directories(${CMAKE_SOURCE_DIR}/Include/Thirdpart/libtorch)
include_directories(${CMAKE_SOURCE_DIR}/Include/Thirdpart/libtorch/torch)
include_directories(${CMAKE_SOURCE_DIR}/Include/Thirdpart/libtorch/torch/csrc/api/include)

include_directories(${CMAKE_SOURCE_DIR}/Include/Thirdpart/TensorRtInclude)
link_directories(${CMAKE_SOURCE_DIR}/Submodules/TPL/Alg/TensorRtLib)

enable_language(CUDA) 
set(CUDNN_DIR /usr/local/)
find_package(CUDA REQUIRED)
message(STATUS "CUDA version:" ${CUDA_VERSION})
include_directories(${CUDA_TOOLKIT_ROOT_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/Modules/VideoAlg)
include_directories(${CMAKE_SOURCE_DIR}/Modules/VideoAlg/Processing/yolov5/include)
include_directories(${CMAKE_SOURCE_DIR}/Modules/VideoAlg/Postprocess)
include_directories(${CMAKE_SOURCE_DIR}/Modules/VideoAlg/Preprocess)

# aux_source_directory(${CMAKE_SOURCE_DIR}/Modules/Common HW_SRC_FILES)
# aux_source_directory(${CMAKE_SOURCE_DIR}/Modules/VideoAlg HW_SRC_FILES)
# aux_source_directory(${CMAKE_SOURCE_DIR}/Modules/VideoAlg/Processing/yolov5/src HW_SRC_FILES)
# aux_source_directory(${CMAKE_SOURCE_DIR}/Modules/VideoAlg/Preprocess HW_SRC_FILES)
# aux_source_directory(${CMAKE_SOURCE_DIR}/Modules/VideoAlg/Postprocess HW_SRC_FILES)\
# 使用不同的变量存储每个目录的源文件
aux_source_directory(${CMAKE_SOURCE_DIR}/Modules/Common COMMON_SRC_FILES)
aux_source_directory(${CMAKE_SOURCE_DIR}/Modules/VideoAlg VideoAlg_SRC_FILES)
aux_source_directory(${CMAKE_SOURCE_DIR}/Modules/VideoAlg/Processing/yolov5/src YOLOV5_SRC_FILES)
aux_source_directory(${CMAKE_SOURCE_DIR}/Modules/VideoAlg/Preprocess PREPROCESS_SRC_FILES)
aux_source_directory(${CMAKE_SOURCE_DIR}/Modules/VideoAlg/Postprocess POSTPROCESS_SRC_FILES)

# 合并所有源文件到一个变量中
set(HW_SRC_FILES
    ${COMMON_SRC_FILES}
    ${VideoAlg_SRC_FILES}
    ${YOLOV5_SRC_FILES}
    ${PREPROCESS_SRC_FILES}
    ${POSTPROCESS_SRC_FILES}
)

# 通用链接库路径
link_directories(${CMAKE_SOURCE_DIR}/Output/Lib)
link_directories(${CMAKE_SOURCE_DIR}/Submodules/TPL)
link_directories(${CMAKE_SOURCE_DIR}/Submodules/TPL/fastdds)
link_directories(${CMAKE_SOURCE_DIR}/Submodules/TPL/Alg)
link_directories(${CMAKE_SOURCE_DIR}/Submodules/TPL/av_opencv)
link_directories(${CMAKE_SOURCE_DIR}/Submodules/TPL/Alg/libtorch)

# 直接编译源文件
aux_source_directory(. HW_SRC_FILES)
add_library(${PROJECT_NAME} SHARED ${HW_SRC_FILES} ExportVideoAlgLib.cpp)

target_link_libraries(${PROJECT_NAME} PRIVATE opencv_core)
target_link_libraries(${PROJECT_NAME} PRIVATE opencv_imgproc)
target_link_libraries(${PROJECT_NAME} PRIVATE opencv_imgcodecs)
target_link_libraries(${PROJECT_NAME} PRIVATE opencv_highgui)
target_link_libraries(${PROJECT_NAME} PRIVATE opencv_calib3d)
target_link_libraries(${PROJECT_NAME} PRIVATE opencv_dnn)
target_link_libraries(${PROJECT_NAME} PRIVATE opencv_features2d)
target_link_libraries(${PROJECT_NAME} PRIVATE opencv_flann)
target_link_libraries(${PROJECT_NAME} PRIVATE opencv_photo)
target_link_libraries(${PROJECT_NAME} PRIVATE opencv_stitching)
target_link_libraries(${PROJECT_NAME} PRIVATE opencv_video)
target_link_libraries(${PROJECT_NAME} PRIVATE opencv_videoio)
target_link_libraries(${PROJECT_NAME} PRIVATE blas)
target_link_libraries(${PROJECT_NAME} PRIVATE lapack)
target_link_libraries(${PROJECT_NAME} PRIVATE yaml-cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE tinyxml2)
target_link_libraries(${PROJECT_NAME} PRIVATE FastddsSer)

target_link_libraries(${PROJECT_NAME} PRIVATE opencv_ml)
target_link_libraries(${PROJECT_NAME} PRIVATE opencv_objdetect)
target_link_libraries(${PROJECT_NAME} PRIVATE stdc++fs)
target_link_libraries(${PROJECT_NAME} PRIVATE nvinfer nvinfer_plugin nvparsers nvonnxparser glog) # TensorRT

