cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(camera_fusion)

find_package(OpenCV 4.1 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

# Executable for create matrix exercise
add_executable (create_matrix src/create_matrix.cpp)
target_link_libraries (create_matrix ${OpenCV_LIBRARIES})

# Executable for change pixels exercise
add_executable (change_pixels src/change_pixels.cpp)
target_link_libraries (change_pixels ${OpenCV_LIBRARIES})

# Executables for load image exercises
add_executable (load_image_1 src/load_image_1.cpp)
target_link_libraries(load_image_1 ${OpenCV_LIBRARIES})

add_executable (load_image_2 src/load_image_2.cpp)
target_link_libraries(load_image_2 ${OpenCV_LIBRARIES})

add_executable (load_image_3 src/load_image_3.cpp)
target_link_libraries(load_image_3 ${OpenCV_LIBRARIES})
