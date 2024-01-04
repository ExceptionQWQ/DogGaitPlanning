cmake_minimum_required(VERSION 3.19)

set(CMAKE_CXX_STANDARD 17)
project(DogGaitPlanning)

find_package(OpenCV 4.8.0 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(main src/main.cpp)
target_link_libraries(main ${OpenCV_LIBRARIES})
