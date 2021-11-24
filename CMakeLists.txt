cmake_minimum_required (VERSION 2.6)
project (libkinova)

include(GNUInstallDirs)

# The version number.
set (libkinova_VERSION_MAJOR 0)
set (libkinova_VERSION_MINOR 1)
set (libkinova_VERSION_PATCH 0)
set (libkinova_VERSION ${libkinova_VERSION_MAJOR}.${libkinova_VERSION_MINOR}.${libkinova_VERSION_PATCH})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -std=c++11")

add_subdirectory (src)
