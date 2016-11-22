# Copyright (c) 2016, Robert Lukierski.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
# 
# Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
# 
# Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.1)

# ------------------------------------------------------------------------------
# Default build configuration
# ------------------------------------------------------------------------------
if(DEFINED CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Build type.")
else()
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type.")
endif()

# ------------------------------------------------------------------------------
# Project name & version
# ------------------------------------------------------------------------------
project(MiscDrivers)
set(${PROJECT_NAME}_VERSION 1.0.0)

# ------------------------------------------------------------------------------
# Enable C++11 and high level warnings
# ------------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 11) 
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -pedantic -Wno-unused-parameter -Wno-unused-variable)

# ------------------------------------------------------------------------------
# Ready for testing
# ------------------------------------------------------------------------------
option(BUILD_TESTS "Enable to build unit tests" ON)
if(BUILD_TESTS)
  enable_testing()
    
  # Code Coverage Build Type
  set(CMAKE_CXX_FLAGS_COVERAGE "-g -O0 --coverage -fprofile-arcs -ftest-coverage" CACHE STRING "Flags used by the C++ compiler during coverage builds." FORCE)
  set(CMAKE_C_FLAGS_COVERAGE "-g -O0 --coverage -fprofile-arcs -ftest-coverage" CACHE STRING "Flags used by the C compiler during coverage builds." FORCE )
  set(CMAKE_EXE_LINKER_FLAGS_COVERAGE "" CACHE STRING "Flags used for linking binaries during coverage builds." FORCE )
  set(CMAKE_SHARED_LINKER_FLAGS_COVERAGE "" CACHE STRING "Flags used by the shared libraries linker during coverage builds." FORCE)
  mark_as_advanced(CMAKE_CXX_FLAGS_COVERAGE CMAKE_C_FLAGS_COVERAGE CMAKE_EXE_LINKER_FLAGS_COVERAGE CMAKE_SHARED_LINKER_FLAGS_COVERAGE)
endif()

include(GenerateExportHeader)

# ---------------------------------------------
# Where to look for Find*.cmake
# ---------------------------------------------
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/scripts)

# ------------------------------------------------------------------------------
# Print Project Info
# ------------------------------------------------------------------------------
message("Project: ${PROJECT_NAME} / ${${PROJECT_NAME}_VERSION}, build type: ${CMAKE_BUILD_TYPE}, compiled on: ${CMAKE_SYSTEM}, flags: ${CMAKE_CXX_FLAGS}")

# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------
# for ASIO (SerialPort)
find_package(Boost COMPONENTS system QUIET)

if(Boost_FOUND)
    find_package(MiscCpp QUIET)
endif()

# for GStreamer
find_package(GLIB QUIET)
if(GLIB_FOUND)
    find_package(LibXml2 QUIET)
    if(LIBXML2_FOUND)
        find_package(GStreamer COMPONENTS app interfaces video QUIET)
    endif()
endif()

# for ViconClient
find_package(Vicon QUIET)
find_package(cereal QUIET)

if(VICON_FOUND)
    find_package(VisionCore QUIET)
    find_package(CameraModels QUIET)
    find_package(TinyXML2 QUIET)
endif()

# ---------------------------------------------
# List of sources
# ---------------------------------------------
set(HEADERS
include/Joystick.hpp
)

set(SOURCES
sources/Joystick.cpp
)

if(Boost_FOUND)
    list(APPEND HEADERS include/SerialPort.hpp)
    list(APPEND SOURCES sources/SerialPort.cpp)
    set(MISCDRIVERS_SUPPORTS_SERIALPORT ON)
    if(MiscCpp_FOUND)
        set(MISCDRIVERS_SUPPORTS_TURTLEBOT ON)
        list(APPEND HEADERS include/TurtlebotDriver.hpp include/TurtlebotClientServer.hpp)
        list(APPEND SOURCES sources/TurtlebotDriver.cpp sources/TurtlebotClientServer.cpp)
    endif()
endif()

if(GSTREAMER_FOUND)
    list(APPEND HEADERS include/GStreamer.hpp)
    list(APPEND SOURCES sources/GStreamer.hpp)
    set(MISCDRIVERS_SUPPORTS_GSTREAMER ON)
endif()

if(VICON_FOUND AND VisionCore_FOUND)
    list(APPEND HEADERS include/ViconClient.hpp)
    list(APPEND SOURCES sources/ViconClient.cpp)
    set(MISCDRIVERS_SUPPORTS_VICON ON)
endif()

# ------------------------------------------------------------------------------
# Build
# ------------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED ${HEADERS} ${SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE sources)
    
if(cereal_FOUND)
    target_link_libraries(${PROJECT_NAME} PUBLIC cereal)
    target_compile_definitions(${PROJECT_NAME} PUBLIC MISC_DRIVERS_HAVE_CEREAL)
endif()
    
if(Boost_FOUND)
    target_link_libraries(${PROJECT_NAME} PRIVATE ${Boost_LIBRARIES})
    target_include_directories(${PROJECT_NAME} PRIVATE ${Boost_INCLUDE_DIRS})
    if(MiscCpp_FOUND)
        target_link_libraries(${PROJECT_NAME} PUBLIC MiscCpp)
    endif()
endif()

if(GSTREAMER_FOUND)
    target_link_libraries(${PROJECT_NAME} PRIVATE  ${GLIB_LIBRARIES} 
                                                   ${GLIB_GOBJECT_LIBRARIES} 
                                                   ${LIBXML2_LIBRARIES} 
                                                   ${GSTREAMER_LIBRARIES} 
                                                   ${GSTREAMER_BASE_LIBRARIES} 
                                                   ${GSTREAMER_APP_LIBRARIES} 
                                                   ${GSTREAMER_INTERFACES_LIBRARIES} 
                                                   ${GSTREAMER_VIDEO_LIBRARIES})
    target_include_directories(${PROJECT_NAME} PRIVATE ${GLIB_INCLUDE_DIRS} 
                                                       ${LIBXML2_INCLUDE_DIR} 
                                                       ${GSTREAMER_INCLUDE_DIRS} 
                                                       ${GSTREAMER_BASE_INCLUDE_DIRS} 
                                                       ${GSTREAMER_APP_INCLUDE_DIRS} 
                                                       ${GSTREAMER_INTERFACES_INCLUDE_DIRS} 
                                                       ${GSTREAMER_VIDEO_INCLUDE_DIRS})
    
    target_compile_definitions(${PROJECT_NAME} PRIVATE ${LIBXML2_DEFINITIONS})
endif()

if(VICON_FOUND AND VisionCore_FOUND)
    target_link_libraries(${PROJECT_NAME} PUBLIC VisionCore)
    
    target_include_directories(${PROJECT_NAME} PRIVATE ${VICON_INCLUDE_DIR})
    target_link_libraries(${PROJECT_NAME} PRIVATE ${VICON_LIBRARY})

    if(CameraModels_FOUND)
        target_link_libraries(${PROJECT_NAME} PUBLIC CameraModels)
        target_compile_definitions(${PROJECT_NAME} PUBLIC MISC_DRIVERS_HAVE_CAMERA_MODELS)
    endif()

    if(TINYXML2_FOUND)
        target_include_directories(${PROJECT_NAME} PRIVATE ${TINYXML2_INCLUDE_DIR})
        target_link_libraries(${PROJECT_NAME} PRIVATE ${TINYXML2_LIBRARY})
        target_compile_definitions(${PROJECT_NAME} PRIVATE MISC_DRIVERS_HAVE_TINYXML)
    endif()
endif()

# ------------------------------------------------------------------------------
# Build executables
# ------------------------------------------------------------------------------
if(MISCDRIVERS_SUPPORTS_TURTLEBOT)
    add_executable(TurtlebotClient sources/TurtlebotClientApp.cpp)
    target_link_libraries(TurtlebotClient PUBLIC ${PROJECT_NAME})
    add_executable(TurtlebotServer sources/TurtlebotServerApp.cpp)
    target_link_libraries(TurtlebotServer PUBLIC ${PROJECT_NAME})
    set(OPT_TARGETS TurtlebotClient TurtlebotServer)
endif()
    
# ------------------------------------------------------------------------------
# Unit Tests
# ------------------------------------------------------------------------------
if(BUILD_TESTS)
    add_subdirectory(tests)
endif()

# ------------------------------------------------------------------------------
# Installation - library
# ------------------------------------------------------------------------------
generate_export_header(${PROJECT_NAME})
set_property(TARGET ${PROJECT_NAME} PROPERTY VERSION ${${PROJECT_NAME}_VERSION})
set_property(TARGET ${PROJECT_NAME} PROPERTY SOVERSION 1)
set_property(TARGET ${PROJECT_NAME} PROPERTY INTERFACE_${PROJECT_NAME}_MAJOR_VERSION 1)
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY COMPATIBLE_INTERFACE_STRING ${PROJECT_NAME}_MAJOR_VERSION)

install(TARGETS ${PROJECT_NAME} ${OPT_TARGETS} EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)
install(FILES ${HEADERS} "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_export.h" DESTINATION include)

# ------------------------------------------------------------------------------
# Installation - generate version file
# ------------------------------------------------------------------------------
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  VERSION ${${PROJECT_NAME}_VERSION}
  COMPATIBILITY AnyNewerVersion
)

# ------------------------------------------------------------------------------
# Installation - Export Targets
# ------------------------------------------------------------------------------
export(EXPORT ${PROJECT_NAME}Targets FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake") 
configure_file(${PROJECT_NAME}Config.cmake.in "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake" @ONLY)
set(ConfigPackageLocation lib/cmake/${PROJECT_NAME})
install(EXPORT ${PROJECT_NAME}Targets FILE ${PROJECT_NAME}Targets.cmake DESTINATION ${ConfigPackageLocation})
install(FILES ${PROJECT_NAME}Config.cmake "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake" DESTINATION ${ConfigPackageLocation})
export(PACKAGE ${PROJECT_NAME})
