cmake_minimum_required(VERSION 2.8)
# cross
if (CMAKE_CROSS_ARM)
  message("cross arm on")
  set(CMAKE_SYSTEM_NAME Linux)
  set(CMAKE_SYSTEM_PROCESSOR arm)
  set(CROSS_PREFIX "arm-linux-gnueabihf-")
  set(CROSS_TOOLCHAIN_PATH /usr/arm-linux-gnueabihf)
  set(CMAKE_INCLUDE_PATH ${CROSS_TOOLCHAIN_PATH}/include)
  set(CMAKE_LIBRARY_PATH ${CROSS_TOOLCHAIN_PATH}/lib)
  set(CMAKE_INSTALL_PREFIX ${CROSS_TOOLCHAIN_PATH}/local)
  # paths
  include_directories(${CROSS_TOOLCHAIN_PATH}/include)
  include_directories(${CROSS_TOOLCHAIN_PATH}/local/include)
  link_directories(${CROSS_TOOLCHAIN_PATH}/lib)
  link_directories(${CROSS_TOOLCHAIN_PATH}/local/lib)
endif (CMAKE_CROSS_ARM)
# subdir
add_subdirectory(ea1)
# SWIG
find_package(SWIG REQUIRED)
include(${SWIG_USE_FILE})
find_package(PythonLibs)
include_directories(${PYTHON_INCLUDE_PATH})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${PROJECT_BINARY_DIR})
# C
set(CMAKE_CXX_FLAGS_RELEASE "-Wall -O2 -fPIC -std=c++11")
#set(CMAKE_CXX_FLAGS_DEBUG "-Wall -g -DEA1_DEBUG -fPIC -std=c++11")
set(CMAKE_CXX_FLAGS_DEBUG "-Wall -g -fPIC -std=c++11")
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif(NOT CMAKE_BUILD_TYPE)
set(PROJECT_NAME probo)
set(PROBO_SRCS probo.cpp probopwm.cpp ppca9685.cpp pgyro.cpp pmpu6050.cpp
  pbbbgpio.cpp)
#set(PROBO_SRCS probo.cpp probopwm.cpp ppca9685.cpp pgyro.cpp)
set (MPU_DIR MPU6050-Pi)
set (MPU_SRCS ${MPU_DIR}/I2Cdev.cpp ${MPU_DIR}/MPU6050.cpp)
set (GPIO_DIR SimpleGPIO)
set (GPIO_SRCS ${GPIO_DIR}/SimpleGPIO.cpp)
# SWIG python
set(PYTHON_IF_SRCS probopy.i)
set_source_files_properties(${PYTHON_IF_SRCS} PROPERTIES  CPLUSPLUS ON)
set_source_files_properties(${PYTHON_IF_SRCS} PROPERTIES  SWIG_FLAGS "-includeall")
set_source_files_properties(${PYTHON_IF_SRCS} PROPERTIES  SWIG_FLAGS "-Wall")
message("${PROJECT_NAME} CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")
# probo core lib
add_library(probo STATIC ${PROBO_SRCS} ${MPU_SRCS} ${GPIO_SRCS})
# PYTHON
swig_add_module(probopy python ${PYTHON_IF_SRCS})
swig_link_libraries(probopy probo ea1 orocos-kdl)
add_custom_target(probopy echo probopy for python)
add_dependencies(probopy ${SWIG_MODULE_probopy_REAL_NAME})
add_custom_command(
  TARGET probopy POST_BUILD
  COMMAND touch __init__.py
  )
# Testing
if(NOT NO_TEST)
  enable_testing()
  add_subdirectory(test)
endif()
