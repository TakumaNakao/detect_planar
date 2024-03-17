project(mathematical_robotics CXX)

cmake_minimum_required(VERSION 3.22)

set(CMAKE_CXX_STANDARD 20) 
add_compile_options(-O3)

include(GNUInstallDirs)

find_package(Eigen3 REQUIRED)
find_package(Matplot++ REQUIRED)
find_package(Open3D REQUIRED)

include_directories(${CERES_INCLUDE_DIR})
include_directories(../utilities)

set(libs 
    Eigen3::Eigen
    Matplot++::matplot
    Open3D::Open3D
)

add_executable(detect_planar "detect_planar.cpp")
target_link_libraries(detect_planar 
    ${libs}
)