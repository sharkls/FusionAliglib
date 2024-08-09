project(FusionAlg)

message("CMAKE_SOURCE_DIR= " ${CMAKE_SOURCE_DIR})
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
include_directories(${CMAKE_SOURCE_DIR}/Include/Thirdpart/cpp_algorithm)
# 在算法编译通过，但是软件链接失败，故加这一行
include_directories(${CMAKE_SOURCE_DIR}/Include/Thirdpart/cpp_algorithm/xflens/cxxlapack/interface)
include_directories(${CMAKE_SOURCE_DIR}/Include/Thirdpart/eigen3)
include_directories(${CMAKE_SOURCE_DIR}/Include/Thirdpart/JSON)
include_directories(${CMAKE_SOURCE_DIR}/Include/Thirdpart/Log)
include_directories(${CMAKE_SOURCE_DIR}/Include/Thirdpart/Log/glog)
include_directories(${CMAKE_SOURCE_DIR}/Include/Thirdpart/fastdds)
include_directories(${CMAKE_SOURCE_DIR}/Include/Thirdpart/yaml-cpp)
include_directories(${CMAKE_SOURCE_DIR}/Modules/Common)

include_directories(${CMAKE_SOURCE_DIR}/Modules/FusionAlg/)
include_directories(${CMAKE_SOURCE_DIR}/Modules/FusionAlg/Preprocess)
include_directories(${CMAKE_SOURCE_DIR}/Modules/FusionAlg/Processing/FusionTracking/include)
include_directories(${CMAKE_SOURCE_DIR}/Modules/FusionAlg/Postprocess)

aux_source_directory(${CMAKE_SOURCE_DIR}/Modules/Common HW_SRC_FILES)
aux_source_directory(${CMAKE_SOURCE_DIR}/Modules/FusionAlg HW_SRC_FILES)
aux_source_directory(${CMAKE_SOURCE_DIR}/Modules/FusionAlg/Processing/FusionTracking/src HW_SRC_FILES)
aux_source_directory(${CMAKE_SOURCE_DIR}/Modules/FusionAlg/Preprocess HW_SRC_FILES)
aux_source_directory(${CMAKE_SOURCE_DIR}/Modules/FusionAlg/Postprocess HW_SRC_FILES)

# 通用链接库路径
link_directories(${CMAKE_SOURCE_DIR}/Output/Lib)
link_directories(${CMAKE_SOURCE_DIR}/Submodules/TPL)
link_directories(${CMAKE_SOURCE_DIR}/Submodules/TPL/fastdds)
link_directories(${CMAKE_SOURCE_DIR}/Submodules/TPL/Alg)

# 直接编译源文件
aux_source_directory(. HW_SRC_FILES)
add_library(${PROJECT_NAME} SHARED ${HW_SRC_FILES} ExportFusionAlgLib.cpp)

# 在算法编译通过，但是软件链接失败，故加这一行
target_link_libraries(${PROJECT_NAME} PUBLIC lapack blas)

target_link_libraries(${PROJECT_NAME} PRIVATE stdc++fs)
target_link_libraries(${PROJECT_NAME} PRIVATE yaml-cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE tinyxml2)
target_link_libraries(${PROJECT_NAME} PRIVATE FastddsSer)
target_link_libraries(${PROJECT_NAME} PRIVATE glog)