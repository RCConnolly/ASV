# Use this file as a reference if you want to use in your program the Fast
# Methods library already installed in your computer

cmake_minimum_required (VERSION 2.6) 
project(test_myFM2)
#project(test)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(fast_methods)

# Not necessary if installed in default locations.
include_directories(${fast_methods_INCLUDE_DIRS})

add_executable(myFM2 myFM2.cpp)
#add_executable(fmm test_fm.cpp)
#add_executable(fm2 test_fm2.cpp)

target_link_libraries(myFM2
    ${fast_methods_LIBRARIES}
)

