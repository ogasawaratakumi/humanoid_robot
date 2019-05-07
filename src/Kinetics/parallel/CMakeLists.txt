cmake_minimum_required(VERSION 2.8)

project( parallel CXX )

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
#target_link_libraries(parallel ${PYTHON_LIBRARIES})
add_executable(parallel Link.cpp Jacobian.cpp Kinematics.cpp main.cpp)
target_link_libraries(parallel ${PYTHON_LIBRARIES})
