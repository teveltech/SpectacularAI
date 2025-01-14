cmake_minimum_required (VERSION 2.8.11)

project (SpectacularAI CXX)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread  -Wall -pedantic -ansi -Wno-unused-result -fPIC")
set(CMAKE_CXX_EXTENSIONS ON) #...with compiler extensions like gnu++11
set(CMAKE_CXX_STANDARD_REQUIRED ON) #...is required...
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(EIGEN3_INCLUDE_DIR "3rdparty/eigen3")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geodesy
        )
include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/Spec_API/Linux_Ubuntu_x86-64/include)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()

add_library(${PROJECT_NAME}
        src/sharedmemtool.cpp
        )

# link_directories(${CMAKE_SOURCE_DIR}/Spec_API/Linux_Ubuntu_x86-64/lib/cmake/)

target_link_libraries(${PROJECT_NAME}
        ${CONAN_LIBS}
        # opencv_calib3d
        opencv_highgui
        ${THREADLIB} ${catkin_LIBRARIES}
  #      spectacularAI
        )

if(CMAKE_COMPILER_IS_GNUCXX OR MINGW OR ${CMAKE_CXX_COMPILER_ID} STREQUAL Clang)
    SET(THREADLIB "pthread")
ENDIF()
