#!/bin/bash

# Set your CUDA installation path
CUDA_PATH=/usr/local/cuda

OPTIX_INCLUDE=/home/saimo/wuyou/NVIDIA-OptiX-SDK-7.1.0-linux64-x86_64/include
OPTIX_SDK_INCLUDE=/home/saimo/wuyou/NVIDIA-OptiX-SDK-7.1.0-linux64-x86_64/SDK
OPTIX_SDK_SUPPORT=/home/saimo/wuyou/NVIDIA-OptiX-SDK-7.1.0-linux64-x86_64/SDK/support
OPTIX_LIB=/home/saimo/wuyou/build/lib

# Set architecture (sm_30 for Kepler, sm_35 for Maxwell, sm_50 for Pascal, etc.)
ARCH=sm_86

# Source files (add your .cpp and .cu files here)
CU_FILES=(RadarRayGenerator.cu RayGenerator.cu)

# Compile CUDA code to object files
for cu_file in "${CU_FILES[@]}"; do
    $CUDA_PATH/bin/nvcc --ptx -I$OPTIX_INCLUDE -I$OPTIX_SDK_INCLUDE -arch=$ARCH -c -o ${cu_file%.cu}.ptx $cu_file -lineinfo
    # $CUDA_PATH/bin/nvcc --optix-ir -I$OPTIX_INCLUDE -I$OPTIX_SDK_INCLUDE -arch=$ARCH -o ${cu_file%.cu}.optixir $cu_file -Xcompiler -fPIC
    # $CUDA_PATH/bin/nvcc -I$OPTIX_INCLUDE -I$OPTIX_SDK_INCLUDE -arch=$ARCH -o ${cu_file%.cu}.o $cu_file -Xcompiler -fPIC
done
