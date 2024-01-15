#!/bin/bash

CUDA_PATH=/usr/local/cuda
# OPTIX=/home/saimo/wuyou/NVIDIA-OptiX-SDK-7.1.0-linux64-x86_64/include
OPTIX_PATH=/home/saimo/wuyou/NVIDIA-OptiX-SDK-7.1.0-linux64-x86_64/SDK
ARCH=sm_86

$CUDA_PATH/bin/nvcc -arch=$ARCH -c -o Calculate.o CalculateLidarRay.cu -Xcompiler -fPIC -I $OPTIX_PATH

ar rcs libcalculateLidarRay.a Calculate.o

rm Calculate.o