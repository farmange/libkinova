cmake_minimum_required (VERSION 2.6)
project (kinovadrv_examples)

# set(CMAKE_VERBOSE_MAKEFILE ON)

include_directories(${CMAKE_SOURCE_DIR}/../include)
set(LIBKINOVADRV_LIBRARIES "${CMAKE_SOURCE_DIR}/../build/src/libkinovadrv.so")

add_executable (kinovadrv_test test.cpp)

target_link_libraries (kinovadrv_test ${LIBKINOVADRV_LIBRARIES})
