############
## Config ##
############

cmake_minimum_required(VERSION 3.0.2)
project(data_packet_solver)

SET(CMAKE_BUILD_TYPE "Release")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

###########
## Build ##
###########

include_directories(
  include
    c
    cpp
)

link_directories(
)

add_executable(${PROJECT_NAME}
  c/DataPacketSolver.c
  test.cpp
)

target_link_libraries(
  ${PROJECT_NAME} 
)