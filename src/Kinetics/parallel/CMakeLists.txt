cmake_minimum_required(VERSION 2.8)

project( parallel CXX )

set(PROJECT_GL simulator_3D)
set(GL_SRC simulator_3D.cpp)

set(PROJECT_TEST parallel)
#set(TEST_SRC main.cpp)

find_package(GLUT)
find_package(OpenGL)
#find_package(Boost COMPONENTS system thread filesystem timer)

add_definitions(-lglut -lGL -lGLU)

add_executable(${PROJECT_GL} ${GL_SRC} Link.cpp Jacobian.cpp Kinematics.cpp)
#add_executable(${PROJECT_TEST} ${TEST_SRC} Link.cpp Jacobian.cpp Kinematics.cpp)

target_link_libraries(${PROJECT_GL} ${GLUT_LIBRARY} ${OPENGL_LIBRARY})
#target_link_libraries(${PROJECT_TEST} ${GLUT_LIBRARY} ${OPENGL_LIBRARY})
