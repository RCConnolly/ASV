cmake_minimum_required (VERSION 2.6) 
project (fast_methods) 

# set a default build type for single-configuration
# CMake generators if no build type is set.
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)

# Select flags.
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g")
set(CMAKE_CXX_FLAGS_RELEASE "-Ofast -fno-finite-math-only")
set(CMAKE_CXX_FLAGS_DEBUG  "-Wall -Wno-unused-local-typedefs -g")

# Finding Boost
find_package(Boost REQUIRED COMPONENTS system filesystem program_options)
if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
else()
    message(FATAL_ERROR "Boost NOT FOUND. Please install it following the instructions on the README file.")
endif()

# Finding CImg
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")
find_package(CImg)
if(CImg_FOUND)
    include_directories(${CIMG_INCLUDE_DIRS})
else()
    message(FATAL_ERROR "CImg NOT FOUND. Please install it following the instructions on the README file.")
endif()

# Finding GeographicLib
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")
find_package(GeographicLib 1.34 REQUIRED)
if(GeographicLib_FOUND)
    include_directories (${GeographicLib_INCLUDE_DIRS})
else()
    message(FATAL_ERROR "GeographicLib NOT FOUND. Please install it following the instructions on the README file.")
endif()

# Self-made includes
include_directories (
    ${Boost_INCLUDE_DIRS}
    ${CImg_INCLUDE_DIRS}
    include
)

# Create main library
add_library(fast_methods SHARED
    src/console/console.cpp
    src/ndgridmap/cell.cpp
    src/ndgridmap/fmcell.cpp
)

# Linking 
target_link_libraries(fast_methods
    ${Boost_LIBRARIES}
    ${CImg_SYSTEM_LIBS}
    ${GeographicLib_LIBRARIES}
)

# Add FM2 capabilities
 add_executable(FM2 examples/currentFM2.cpp)

target_link_libraries(FM2 fast_methods)

# Install section
# We are not using CMakePackageConfigHelpers because CIMG variables are lists
# and those functions do not manage them properly.
configure_file(
    cmake/fast_methods-config.cmake.in
    ${CMAKE_BINARY_DIR}/fast_methods-config.cmake
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/fast_methods-config.cmake
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/cmake/fast_methods
)

install(TARGETS fast_methods
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)


install(DIRECTORY scripts
    DESTINATION ${CMAKE_INSTALL_PREFIX}/share/fast_methods)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_PREFIX}/include
    FILES_MATCHING
        PATTERN "*.hpp"
        PATTERN "*.h"
)

# Uninstall target
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
