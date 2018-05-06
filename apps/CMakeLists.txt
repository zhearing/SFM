cmake_minimum_required(VERSION 2.6)

SET(CMAKE_CXX_FLAGS_RELEASE "-std=c++11")
SET(CMAKE_CXX_FLAGS_DEBUG "-g")
SET(CMAKE_BUILD_TYPE RELEASE)
INCLUDE_DIRECTORIES(/usr/local/include)

ADD_EXECUTABLE(reconstruction main)
TARGET_LINK_LIBRARIES(reconstruction ${CORELIBS})


