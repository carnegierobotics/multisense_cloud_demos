# @file CMakeLists.txt
#
# Top level build instructions.
#
# Significant history (date, user, job code, action):
#   2015-05-21, dlr@carnegierobotics.com, PR1044, Created file, based on
#               original CMakeLists.txt from Dan Tascione.

#
# Make sure there is a reasonable version of CMAKE installed.
#

cmake_minimum_required(VERSION 2.8)
project(multisense_cloud_demos)

#
# Turn on optimizations and maximum warnings.
#
add_compile_options(-Wall -Wextra -Werror -Wpedantic -Wno-variadic-macros)

# This will be useful for build rules in subordinate CMakeLists files.

set(BASE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

# Define where the result of the build should go.

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BASE_DIRECTORY}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${BASE_DIRECTORY}/lib)

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Selecting default build type: Release")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default: Release)" FORCE)
endif ()

# Include (-I) subdirectories and find packages.

find_package(OpenCV)
include_directories(BEFORE external/LibMultiSense/source ${BASE_DIRECTORY}/include)

# Dispatch to subordinate CMakeList.txt files.

add_subdirectory(external/LibMultiSense/source/LibMultiSense)
add_subdirectory(src)
