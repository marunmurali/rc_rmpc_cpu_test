cmake_minimum_required(VERSION 2.8.3)
project(rc_rmpc_cpu)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
roscpp
std_msgs
sensor_msgs
message_generation
custom_pkg
)

add_message_files(
  DIRECTORY msg
  FILES
  control_signal.msg
  log_sig.msg
)

catkin_package(
CATKIN_DEPENDS
roscpp
std_msgs
INCLUDE_DIRS include
)

set(MIN_CUDA_SRC
  src/main.cpp
)

include_directories(
include
/home/arun/eigen
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
)

add_executable(rc_rmpc_cpu
  ${MIN_CUDA_SRC}
src/Car.cpp src/MathTools.cpp src/Sensing.cpp src/main.cpp src/Jcpu.cpp
)

add_dependencies(rc_rmpc_cpu sensor_msgs_generate_messages_cpp)

target_link_libraries(rc_rmpc_cpu
${catkin_LIBRARIES})
