CMAKE_MINIMUM_REQUIRED(VERSION 2.91.0)
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

PROJECT(LoboFEM)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# find eigen
set(EIGEN3_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/external/eigen-git-mirror)
find_package(Eigen3 REQUIRED)

# find imgui
# 3rd party libraries
include(cmake/3rdparty.cmake)
# macro and configurations
include(cmake/config.cmake)

#SET(CMAKE_CXX_COMPILER "g++")

SET( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin) 

SET( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

LINK_DIRECTORIES( ${PROJECT_SOURCE_DIR}/lib)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/LoboFEM)


ADD_SUBDIRECTORY( ${PROJECT_SOURCE_DIR}/src)

