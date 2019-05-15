cmake_minimum_required(VERSION 2.8)

project( parallel CXX )

#find_package(PythonLibs 2.7)
find_package(PythonLibs REQUIRED)
#target_include_directories(parallel PRIVATE ${PYTHON_INCLUDE_DIRS})
#target_link_libraries(parallel ${PYTHON_LIBRARIES})
include_directories(${PYTHON_INCLUDE_DIRS})
add_executable(parallel Link.cpp Jacobian.cpp Kinematics.cpp main.cpp)
target_link_libraries(parallel ${PYTHON_LIBRARIES})
