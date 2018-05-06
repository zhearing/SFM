PROJECT(Sparse3D)
cmake_minimum_required(VERSION 2.6)

#for templates you need to add additional flags C++11  -std=c0x
SET(CMAKE_CXX_FLAGS_RELEASE "-std=c++11 -w -Wall -O3 -ffast-math -fomit-frame-pointer -funroll-loops -fopenmp -msse2")
#SET(CMAKE_CXX_FLAGS_DEBUG "-g")
SET(CMAKE_BUILD_TYPE RELEASE)

find_package( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})
set(CORELIBS Sparse3D ${OpenCV_LIBS})

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(apps)
