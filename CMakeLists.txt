cmake_minimum_required(VERSION 3.20)
project(handeye-calibration)

set(CMAKE_CXX_STANDARD 14)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
add_executable(handeye-calibration main.cpp calibrate_eih.cpp calibrate_eih.h)
target_link_libraries (handeye-calibration Eigen3::Eigen)