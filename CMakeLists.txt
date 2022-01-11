#  @author dingyi
#  @start_date 10/26/2021
#  @modified_date 01/10/2022
#  @calibrate_handeye

cmake_minimum_required(VERSION 3.20) # CMake version check
project(handeye-calibration)         # Create project
set(CMAKE_CXX_STANDARD 14)           # Enable c++14 standard

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
add_executable(handeye-calibration main.cpp calibrate_eih.cpp calibrate_eih.h)
target_link_libraries (handeye-calibration Eigen3::Eigen)