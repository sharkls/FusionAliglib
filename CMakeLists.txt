# 需要的CMake的最小版本号
cmake_minimum_required(VERSION 3.10)

# 执行文件另取别名
project(testAlgLib)
set(CMAKE_BUILD_TYPE Release)       # 设置编译模式
if(NOT DEFINED CMAKE_PLATFORM)      # 设置编译平台
    set(CMAKE_PLATFORM 86)
else()
    set(CMAKE_PLATFORM ${CMAKE_PLATFORM})
endif()
set(CMAKE_CXX_STANDARD 14)          # 设置C++版本
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_ARCHITECTURES 60)
add_compile_options(-shared)
add_definitions(-w)   # 不显示warning

message("CMAKE_SOURCE_DIR= " ${CMAKE_SOURCE_DIR})

# 设置输出路径
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/Output/Lib) #动态库 
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/Output/Lib) #静态库 
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/Output/) #执行文件 

if(CMAKE_PLATFORM MATCHES "86")
    enable_language(CUDA)       # 启用 CUDA 语言支持
else()
endif()

# 包含头文件的路径
include_directories(${CMAKE_SOURCE_DIR}/Submodules/FastddsSer/data)
include_directories(${CMAKE_SOURCE_DIR}/Submodules/FastddsSer/param)
include_directories(${CMAKE_SOURCE_DIR}/Include/AlgParam)
include_directories(${CMAKE_SOURCE_DIR}/Modules)
include_directories(${CMAKE_SOURCE_DIR}/Include)
include_directories(${CMAKE_SOURCE_DIR}/Include/Interface)
include_directories(${CMAKE_SOURCE_DIR}/Include/Common)
include_directories(${CMAKE_SOURCE_DIR}/Include/Thirdpart)
include_directories(${CMAKE_SOURCE_DIR}/Include/Thirdpart/fastdds)
include_directories(${CMAKE_SOURCE_DIR}/Include/Thirdpart/av_opencv)
include_directories(${CMAKE_SOURCE_DIR}/Include/Thirdpart/cpp_algorithm)
include_directories(${CMAKE_SOURCE_DIR}/Include/ThirdPart/tiny_xml)
include_directories(${CMAKE_SOURCE_DIR}/Include/Thirdpart/eigen3)
include_directories(${CMAKE_SOURCE_DIR}/Include/Thirdpart/Log/glog)
include_directories(${CMAKE_SOURCE_DIR}/Include/Thirdpart/SSCN)
include_directories(${CMAKE_SOURCE_DIR}/Include/Thirdpart/SSCN/spconv)
include_directories(${CMAKE_SOURCE_DIR}/Include/Thirdpart/SSCN/spconv2)
include_directories(${CMAKE_SOURCE_DIR}/Include/Thirdpart/yaml-cpp)
include_directories(${CMAKE_SOURCE_DIR}/Include/Thirdpart/libtorch)
include_directories(${CMAKE_SOURCE_DIR}/Include/Thirdpart/libtorch/torch/csrc/api/include)
include_directories(${CMAKE_SOURCE_DIR}/Include/Thirdpart/JSON)
include_directories(${CMAKE_SOURCE_DIR}/Include/Thirdpart/TensorRtInclude)

# 链接动态库的路径
link_directories(${CMAKE_SOURCE_DIR}/Output/Lib)
link_directories(${CMAKE_SOURCE_DIR}/Submodules/TPL)
link_directories(${CMAKE_SOURCE_DIR}/Submodules/TPL/Alg)
link_directories(${CMAKE_SOURCE_DIR}/Submodules/TPL/fastdds)
link_directories(${CMAKE_SOURCE_DIR}/Submodules/TPL/Alg/TensorRtLib)

if(CMAKE_PLATFORM MATCHES "86")
    link_directories(${CMAKE_SOURCE_DIR}/Submodules/TPL/Alg/libtorch)
    link_directories(${CMAKE_SOURCE_DIR}/Submodules/TPL/av_opencv)
else()
endif()

if(CMAKE_PLATFORM MATCHES "220")
    link_directories(${CMAKE_SOURCE_DIR}/Submodules/TPL/CNStream)
    link_directories(${CMAKE_SOURCE_DIR}/Submodules/TPL/cncv)
    link_directories(${CMAKE_SOURCE_DIR}/Submodules/TPL/cambricon)
else()
endif()

if(CMAKE_PLATFORM MATCHES "11")
    link_directories(${CMAKE_SOURCE_DIR}/Submodules/TPL/fservo)
    link_directories(${CMAKE_SOURCE_DIR}/Submodules/TPL/aarch64-xilinx-linux/usr/lib)
    link_directories(${CMAKE_SOURCE_DIR}/Submodules/TPL/aarch64-xilinx-linux/lib)
else()
endif()

# 子目录的路径
add_subdirectory(Modules)
add_subdirectory(Submodules/FastddsSer)

# 源文件的路径
aux_source_directory(. HW_SRC_FILES)

if(CMAKE_PLATFORM MATCHES "86")
    #生成可执行程序
    add_executable(${PROJECT_NAME} testX86.cpp)
    target_link_libraries(${PROJECT_NAME} PRIVATE opencv_core)
    target_link_libraries(${PROJECT_NAME} PRIVATE opencv_imgproc)
    target_link_libraries(${PROJECT_NAME} PRIVATE opencv_imgcodecs)
    target_link_libraries(${PROJECT_NAME} PRIVATE opencv_highgui)
    target_link_libraries(${PROJECT_NAME} PRIVATE opencv_calib3d)
    target_link_libraries(${PROJECT_NAME} PRIVATE opencv_dnn)
    target_link_libraries(${PROJECT_NAME} PRIVATE opencv_features2d)
    target_link_libraries(${PROJECT_NAME} PRIVATE opencv_flann)
    target_link_libraries(${PROJECT_NAME} PRIVATE opencv_ml)
    target_link_libraries(${PROJECT_NAME} PRIVATE opencv_objdetect)
    target_link_libraries(${PROJECT_NAME} PRIVATE opencv_photo)
    target_link_libraries(${PROJECT_NAME} PRIVATE opencv_stitching)
    target_link_libraries(${PROJECT_NAME} PRIVATE opencv_video)
    target_link_libraries(${PROJECT_NAME} PRIVATE opencv_videoio)
    target_link_libraries(${PROJECT_NAME} PRIVATE yaml-cpp)
    target_link_libraries(${PROJECT_NAME} PRIVATE tinyxml2)
    target_link_libraries(${PROJECT_NAME} PRIVATE stdc++fs)
    target_link_libraries(${PROJECT_NAME} PRIVATE nvinfer nvinfer_plugin nvparsers nvonnxparser) # TensorRT

    target_link_libraries(${PROJECT_NAME} PRIVATE glog)
    target_link_libraries(${PROJECT_NAME} PRIVATE cuhash)
    target_link_libraries(${PROJECT_NAME} PRIVATE FastddsSer)

    # 链接各算法模块的动态库
    # target_link_libraries(${PROJECT_NAME} PRIVATE PcAlg)
    target_link_libraries(${PROJECT_NAME} PRIVATE VideoAlg)
    # target_link_libraries(${PROJECT_NAME} PRIVATE FusionAlg)
    # target_link_libraries(${PROJECT_NAME} PRIVATE SceneAlg)
    # target_link_libraries(${PROJECT_NAME} PRIVATE EventAlg)
    # target_link_libraries(${PROJECT_NAME} PRIVATE TriggerAlg)
    # target_link_libraries(${PROJECT_NAME} PRIVATE 2K1VAlg)
    target_link_libraries(${PROJECT_NAME} PRIVATE TestAlg)

elseif(CMAKE_PLATFORM MATCHES "1046")
    # 生成可执行程序
    add_executable(${PROJECT_NAME} test1046.cpp)

    target_link_libraries(${PROJECT_NAME} PRIVATE yaml-cpp)
    target_link_libraries(${PROJECT_NAME} PRIVATE tinyxml2)
    target_link_libraries(${PROJECT_NAME} PRIVATE stdc++fs)
    target_link_libraries(${PROJECT_NAME} PRIVATE glog)
    target_link_libraries(${PROJECT_NAME} PRIVATE FastddsSer)

    # 链接各算法模块的动态库
    target_link_libraries(${PROJECT_NAME} PRIVATE FusionAlg)
    target_link_libraries(${PROJECT_NAME} PRIVATE SceneAlg)
    target_link_libraries(${PROJECT_NAME} PRIVATE EventAlg)
    target_link_libraries(${PROJECT_NAME} PRIVATE TriggerAlg)
elseif(CMAKE_PLATFORM MATCHES "220")
    add_executable(${PROJECT_NAME} test220.cpp)
    target_link_libraries(${PROJECT_NAME} PRIVATE opencv_core)
    target_link_libraries(${PROJECT_NAME} PRIVATE opencv_imgproc)
    target_link_libraries(${PROJECT_NAME} PRIVATE opencv_imgcodecs)
    target_link_libraries(${PROJECT_NAME} PRIVATE yaml-cpp)
    target_link_libraries(${PROJECT_NAME} PRIVATE FastddsSer)
    target_link_libraries(${PROJECT_NAME} PRIVATE VideoAlg)
endif()