cmake_minimum_required (VERSION 3.1)
project(octree)

#--------------------
# DEPENDENCY: eigen 3
#--------------------
find_path(EIGEN_INCLUDE_DIR Eigen/Core
	/usr/local/include/eigen3
	/usr/include/eigen3
)
message("Please use C++11 to compile octree")
set (CMAKE_CXX_STANDARD 11)

include_directories(${CMAKE_CURRENT_BINARY_DIR} ${EIGEN_INCLUDE_DIR})

add_definitions(-O3)

add_executable(${PROJECT_NAME} main.cpp)

target_link_libraries(${PROJECT_NAME} pthread)
