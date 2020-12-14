cmake_minimum_required (VERSION 2.6)

# include directory containing header files
include_directories (${libkinova_SOURCE_DIR}/include)

# set as shared library
add_library (kinovadrv SHARED kinovadrv.cpp logger.cpp)

if (DEBUG)
MESSAGE(STATUS "Debug mode. All message will be stub with empty (zero) values.")
add_definitions(-DDEBUG_STUB_MSG -shared)
endif()

set_target_properties(
  kinovadrv
  PROPERTIES
    SOVERSION ${libkinova_VERSION_MAJOR}
    VERSION ${libkinova_VERSION}
)

# # link external libraries
# target_link_libraries (kinovadrv ${LIBUSB_1_LIBRARIES})
# target_link_libraries (kinovadrv ${Boost_LIBRARIES})

# set the install targets
# install (TARGETS kinovadrv LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}")

# file (GLOB kinovadrv_header_files ${libkinova_SOURCE_DIR}/include/*.h)
# install (FILES ${kinovadrv_header_files} DESTINATION include/libkinova)
