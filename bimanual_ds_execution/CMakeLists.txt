cmake_minimum_required(VERSION 2.8.3)
project(bimanual_ds_execution)

find_package(catkin REQUIRED COMPONENTS roscpp mathlib bimanual_ds)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS mathlib bimanual_ds
)

FILE(GLOB SRC_FILES src/*.cpp)

include_directories(include ${catkin_INCLUDE_DIRS})

#common commands for building c++ executables and libraries
add_library(${PROJECT_NAME} ${SRC_FILES})
