cmake_minimum_required(VERSION 3.10)
project(MyCudaProject)

# Find CUDA
# cmake会自动寻找CUDA位置
find_package(CUDA REQUIRED)

set(Optix_INCLUDE /home/saimo/wuyou/NVIDIA-OptiX-SDK-7.1.0-linux64-x86_64/include
                  /home/saimo/wuyou/NVIDIA-OptiX-SDK-7.1.0-linux64-x86_64/SDK
                  /home/saimo/wuyou/NVIDIA-OptiX-SDK-7.1.0-linux64-x86_64/SDK/support)

set(COMMON_INCLUDE /home/saimo/wuyou/project/optix/common)

set(CalculateLidarRay_INCLUDE /home/saimo/wuyou/project/optix/cuda)

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -arch=sm_86)

include_directories(${CUDA_INCLUDE_DIRS} 
                    ${Optix_INCLUDE}
                    ${COMMON_INCLUDE}
                    ${CalculateLidarRay_INCLUDE})

set(SRC main.cpp 
        Lidar.cpp
        LidarData.cpp
        SensorData.cpp
        RayGeneratorClass.cpp 
        LidarRayGenerator.cpp)

add_executable(test ${SRC})

target_link_libraries(test
    # 此处填入你的静态库路径
    # /home/saimo/wuyou/project/optix/liboptix.a
    /home/saimo/wuyou/project/optix/cuda/libcalculateLidarRay.a
    /home/saimo/wuyou/build/lib/libglad.so
    /home/saimo/wuyou/build/lib/libsutil_7_sdk.so
    # cmake会自动加入所需要的其他cuda库
    ${CUDA_LIBRARIES}
    )
