cmake_minimum_required(VERSION 2.8)
project(DBoW2)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

IF(NOT DBOW2_LIB_TYPE)
  SET(DBOW2_LIB_TYPE "STATIC") # "SHARED"
ENDIF(NOT DBOW2_LIB_TYPE)

MESSAGE(STATUS "BUILD TYPE:" ${CMAKE_BUILD_TYPE})

# Check architecture
IF(${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm")
    add_definitions("-DENABLE_SSE")
    MESSAGE("SSE flags: " ${SSE_FLAGS})
    # Optimized for Jetson TX2 only; refer to https://devtalk.nvidia.com/default/topic/1028179/jetson-tx2/gcc-options-for-tx2
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}     -Wall -O3 -ffast-math -flto -march=armv8-a+crypto -mcpu=cortex-a57+crypto")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -ffast-math -flto -march=armv8-a+crypto -mcpu=cortex-a57+crypto")
ELSE()
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}     -Wall -O3 -march=native ")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native")
ENDIF()

set(HDRS_DBOW2
  DBoW2/BowVector.h
  DBoW2/FORB.h 
  DBoW2/FClass.h       
  DBoW2/FeatureVector.h
  DBoW2/ScoringObject.h   
  DBoW2/TemplatedVocabulary.h)
set(SRCS_DBOW2
  DBoW2/BowVector.cpp
  DBoW2/FORB.cpp      
  DBoW2/FeatureVector.cpp
  DBoW2/ScoringObject.cpp)

set(HDRS_DUTILS
  DUtils/Random.h
  DUtils/Timestamp.h)
set(SRCS_DUTILS
  DUtils/Random.cpp
  DUtils/Timestamp.cpp)

 # customize opencv dir
if ($ENV{ROS_DISTRO} STREQUAL "indigo")
     set( OpenCV_DIR /opt/opencv2/share/OpenCV )
     find_package(OpenCV 2.4.13 REQUIRED core ocl)
elseif ($ENV{ROS_DISTRO} STREQUAL "kinetic")
     set( OpenCV_DIR /opt/opencv3/share/OpenCV )
     find_package(OpenCV 3.4.1 REQUIRED
                PATHS /opt/opencv3
                NO_DEFAULT_PATH)
elseif ($ENV{ROS_DISTRO} STREQUAL "noetic")
   find_package(OpenCV 4 REQUIRED)
endif()

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${OpenCV_INCLUDE_DIRS})
add_library(DBoW2 ${DBOW2_LIB_TYPE} ${SRCS_DBOW2} ${SRCS_DUTILS})
target_link_libraries(DBoW2 ${OpenCV_LIBS})

