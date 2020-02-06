cmake_minimum_required(VERSION 2.8.3)
project(potential_planning)

add_compile_options(-std=c++11)

find_package(PythonLibs 2.7)
find_package(Threads REQUIRED)

include_directories(
        ${catkin_INCLUDE_DIRS}
        /usr/include
        /usr/include/c++/7.4.0
        /usr/include/c++/7/backward
        /usr/include/x86_64-linux-gnu
        /usr/include/x86_64-linux-gnu/c++/7.4.0
        /usr/include/eigen3
        include
)

add_executable(potential_planning
        src/main.cpp
        include/matplotlibcpp.h
        include/object.h)

target_include_directories(potential_planning PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(potential_planning ${PYTHON_LIBRARIES})