cmake_minimum_required(VERSION 3.0.2)
project(ampc_cpp)

## Specify C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
find_package(Eigen3 REQUIRED)

## 修复 qpOASES 查找逻辑
# 方法1：尝试使用 find_package
find_package(qpOASES QUIET)

if(NOT qpOASES_FOUND)
    # 方法2：手动设置 qpOASES 路径
    # set(QPOASES_ROOT_DIR "/home/pku-yifei/qpOASES" CACHE PATH "qpOASES root directory")
    
    # 查找头文件
    find_path(QPOASES_INCLUDE_DIR
        NAMES qpOASES.hpp
        PATHS
            /home/pku-yifei/qpOASES/include
            /usr/local/include
            /usr/include
            /opt/qpOASES/include
        PATH_SUFFIXES qpOASES
    )
    
    # 查找库文件
    find_library(QPOASES_LIBRARY
        NAMES qpOASES libqpOASES
        PATHS
            /home/pku-yifei/qpOASES/bin
            /home/pku-yifei/qpOASES/lib
            /home/pku-yifei/qpOASES/build/libs
            /home/pku-yifei/qpOASES/bin
            /home/pku-yifei/qpOASES/lib
            /home/pku-yifei/qpOASES/build/libs
            /usr/local/lib
            /usr/lib
            /opt/qpOASES/lib
    )
    
    # 检查是否找到
    if(QPOASES_INCLUDE_DIR AND QPOASES_LIBRARY)
        set(qpOASES_FOUND TRUE)
        set(QPOASES_INCLUDE_DIRS /home/pku-yifei/qpOASES/include)
        set(QPOASES_LIBRARIES /home/pku-yifei/qpOASES/build/libs/libqpOASES.so)
        message(STATUS "qpOASES found manually:")
        message(STATUS "  Include dirs: ${QPOASES_INCLUDE_DIRS}")
        message(STATUS "  Libraries: ${QPOASES_LIBRARIES}")
    else()
        message(FATAL_ERROR "qpOASES not found! Please install qpOASES or set QPOASES_ROOT_DIR")
    endif()
else()
    # 如果通过 find_package 找到，设置变量
    set(QPOASES_INCLUDE_DIRS ${qpOASES_INCLUDE_DIRS})
    set(QPOASES_LIBRARIES ${qpOASES_LIBRARIES})
    message(STATUS "qpOASES found via find_package:")
    message(STATUS "  Include dirs: ${QPOASES_INCLUDE_DIRS}")
    message(STATUS "  Libraries: ${QPOASES_LIBRARIES}")
endif()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ampc_cpp
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  /home/pku-yifei/qpOASES/include  # 添加 qpOASES 包含目录
)

## Declare C++ libraries
add_library(adaptive src/adaptive.cpp)
add_library(mpc src/mpc.cpp)
add_library(kalman_filter src/kalman_filter.cpp)
add_library(matrix_utils src/matrix_utils.cpp)
add_library(ros_interface src/ros_interface.cpp)

## Declare a C++ executable
add_executable(auv_control_node 
  src/main_node.cpp
  src/kalman_filter.cpp
  src/adaptive.cpp
  src/mpc.cpp
  src/matrix_utils.cpp
  src/ros_interface.cpp
)

## Add cmake target dependencies of the executable
add_dependencies(auv_control_node ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(auv_control_node
  adaptive
  mpc
  kalman_filter
  matrix_utils
  ros_interface
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  /home/pku-yifei/qpOASES/build/libs/libqpOASES.so  # 链接 qpOASES 库
)

# 调试信息
message(STATUS "=== Build Configuration ===")
message(STATUS "Catkin include directories: ${catkin_INCLUDE_DIRS}")
message(STATUS "Eigen3 include directories: ${EIGEN3_INCLUDE_DIRS}")
message(STATUS "qpOASES include directories: ${QPOASES_INCLUDE_DIRS}")
message(STATUS "qpOASES libraries: ${QPOASES_LIBRARIES}")
message(STATUS "===============================")

#############
## Install ##
#############

## Mark executable for installation
# install(TARGETS auv_control_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark cpp header files for installation
# install(
#   DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
# )