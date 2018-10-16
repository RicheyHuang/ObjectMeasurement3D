cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

project(ObjectMeasurement3D)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(SOURCE_FILES src/ObjectMeasurement3D.cpp)

add_executable(ObjectMeasurement3D ${SOURCE_FILES})
target_link_libraries(ObjectMeasurement3D ${PCL_LIBRARIES})