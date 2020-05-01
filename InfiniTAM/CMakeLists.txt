##########################################
# Top-level CMakeLists.txt for InfiniTAM #
##########################################

cmake_minimum_required(VERSION 3.10)

PROJECT(FastFusion_v2)

#################################
# Specify the CMake module path #
#################################

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")
set(CMAKE_CXX_STANDARD 11)

FIND_PACKAGE(OpenCV 2.4 REQUIRED)
FIND_PACKAGE(Eigen REQUIRED)
include_directories(/usr/include/eigen3)
#include_directories(${EIGEN3_INCLUDE_DIR})
FIND_PACKAGE(kindr REQUIRED)
FIND_PACKAGE(Glog REQUIRED)
find_package(PkgConfig)
PKG_CHECK_MODULES(YamlCpp REQUIRED yaml-cpp>0.5)

#################################
# Add additional compiler flags #
#################################

IF(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libstdc++ -Wno-deprecated-declarations -Wno-unused-function")
ENDIF()

IF(NOT MSVC_IDE)
  SET(CFLAGS_WARN "-Wall -Wextra -Wno-unused-parameter -Wno-strict-aliasing")
#  SET(CMAKE_CXX_FLAGS "-fPIC -O3 -march=native ${CFLAGS_WARN} ${CMAKE_CXX_FLAGS} -std=c++11 -lstdc++")
  SET(CMAKE_CXX_FLAGS "-fPIC -O3 ${CFLAGS_WARN} ${CMAKE_CXX_FLAGS} -std=c++11 -lstdc++")
  #SET(CMAKE_CXX_FLAGS "-fPIC -g ${CFLAGS_WARN} ${CMAKE_CXX_FLAGS}")
ENDIF()


######################
# Add subdirectories #
######################

ADD_SUBDIRECTORY(ROVIO)
ADD_SUBDIRECTORY(Apps)
ADD_SUBDIRECTORY(FernRelocLib)
ADD_SUBDIRECTORY(InputSource)
ADD_SUBDIRECTORY(ITMLib)
ADD_SUBDIRECTORY(MiniSlamGraphLib)
ADD_SUBDIRECTORY(ORUtils)

