cmake_minimum_required (VERSION 2.6)
project (libkinova)

# The version number.
set (libkinova_VERSION_MAJOR 0)
set (libkinova_VERSION_MINOR 1)
set (libkinova_VERSION_PATCH 0)
set (libkinova_VERSION ${libkinova_VERSION_MAJOR}.${libkinova_VERSION_MINOR}.${libkinova_VERSION_PATCH})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror")

# add custom CMAKE_MODULE_PATH for our provided modules
# set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

# detect correct lib-suffix
# include (libkinova_detect_lib_suffix)

# Compile examples?
#option (BUILD_EXAMPLES "Build examples?" OFF)

# find libusb library
# find_package (libusb-1.0 REQUIRED)
# include_directories (${LIBUSB_1_INCLUDE_DIRS})

# find boost-system
# find_package (Boost REQUIRED COMPONENTS system)
# include_directories (${Boost_INCLUDE_DIRS})

# add subdirectory containing source files
add_subdirectory (src)

# configure and install pkg-config file
# configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/kindrv.pc.in" "${CMAKE_CURRENT_BINARY_DIR}/libkinova.pc" @ONLY)
# install (FILES "${CMAKE_CURRENT_BINARY_DIR}/libkinova.pc" DESTINATION "${CMAKE_INSTALL_LIBDIR}/pkgconfig")

# install udev-rule
# option (INSTALL_UDEV "Install udev-rule? Needs root privileges, as it writes to '/etc' directory." ON)
# set (UDEV_INSTALL_DIR "/etc/udev/rules.d" CACHE STRING "Destination for udev rules")
# if (INSTALL_UDEV)
#   install (FILES "10-libkinova.rules" DESTINATION ${UDEV_INSTALL_DIR} OPTIONAL)
# endif()

# uninstall target
# configure_file("${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/cmake_uninstall.cmake.in" "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake" IMMEDIATE @ONLY)
# add_custom_target(uninstall COMMAND "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")