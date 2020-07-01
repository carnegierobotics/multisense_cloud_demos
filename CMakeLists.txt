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

# This will be useful for build rules in subordinate CMakeLists files.

set(BASE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

# Define where the result of the build should go.

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BASE_DIRECTORY}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${BASE_DIRECTORY}/lib)

#
# Turn on optimizations and maximum warnings.
#

# Turn on debugging symbols / optimizations and warnings.  To choose
# which build type, use "cmake -DCMAKE_BUILD_TYPE=Debug <directory>" or
# "cmake -DCMAKE_BUILD_TYPE=Release <directory>".

set(CMAKE_C_FLAGS_DEBUG "-O0 -g -march=native -msse4.2 -Wall -fstrict-aliasing -pthread -fopenmp")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -march=native -msse4.2 -Wall -fstrict-aliasing -pthread -fopenmp")

set(CMAKE_C_FLAGS_RELEASE "-O2 -ffunction-sections -fomit-frame-pointer -march=native -msse4.2 -mfpmath=both -Wall -fstrict-aliasing -pthread -fopenmp")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -ffunction-sections -fomit-frame-pointer -march=native -msse4.2 -mfpmath=both -Wall -fstrict-aliasing -pthread -fopenmp")

# Include (-I) subdirectories and find packages.

find_package(OpenCV)
include_directories(BEFORE external/LibMultiSense/source ${BASE_DIRECTORY}/include)

# Dispatch to subordinate CMakeList.txt files.

add_subdirectory(external/LibMultiSense/source/LibMultiSense)
add_subdirectory(src)
