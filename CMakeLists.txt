cmake_minimum_required(VERSION 3.10)
project(kf_lib)

set(CMAKE_CXX_STANDARD 11)

    # Add include dirs
include_directories(${PROJECT_SOURCE_DIR}/include)

    # Packages
find_package(Eigen3 REQUIRED)

    # Add source files
file(GLOB_RECURSE SOURCES "src/*.cpp")

    # Generate Library
add_library(${PROJECT_NAME} ${SOURCES})

    # Link to External library(If needed)
# target_link_libraries(${PROJECT_NAME} <external_lib>)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)