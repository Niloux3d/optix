#!/bin/bash

CUDA_PATH=/usr/local/cuda
ARCH=sm_86

$CUDA_PATH/bin/nvcc -arch=$ARCH -c -o Calculate.o CalculateLidarRay.cu -Xcompiler -fPIC

ar rcs libcalculateLidarRay.a Calculate.o

rm Calculate.o