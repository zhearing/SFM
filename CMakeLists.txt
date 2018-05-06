PROJECT(Sparse3D)
cmake_minimum_required(VERSION 2.6)

SET(CMAKE_CXX_FLAGS_RELEASE "-std=c++11")
#SET(CMAKE_CXX_FLAGS_DEBUG "-g")
SET(CMAKE_BUILD_TYPE RELEASE)

find_package( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})
set(CORELIBS Sparse3D ${OpenCV_LIBS})

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(apps)