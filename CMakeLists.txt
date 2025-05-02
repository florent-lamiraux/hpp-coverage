# Copyright 2025 CNRS
# Author: Florent Lamiraux
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:

# 1. Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

cmake_minimum_required(VERSION 3.10)

set(CUSTOM_HEADER_DIR hpp/coverage)
set(CXX_DISABLE_WERROR TRUE)
set(DOXYGEN_USE_MATHJAX YES)
set(DOXYGEN_USE_TEMPLATE_CSS TRUE)
set(PROJECT_USE_CMAKE_EXPORT TRUE)

set(PROJECT_NAME hpp-coverage)
set(PROJECT_DESCRIPTION "Models and algorithms for trajectory generation in JERICO project")

include(cmake/hpp.cmake)
include(cmake/idl.cmake)
include(cmake/python.cmake)

option(CORBA "Implement CORBA server and client" ON)

compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

list(APPEND PKG_CONFIG_ADDITIONAL_VARIABLES cmake_plugin)

add_project_dependency(hpp-manipulation REQUIRED)
add_project_dependency(staubli_tx2_90_support REQUIRED)

# Library
set(${PROJECT_NAME}_HEADERS
  include/hpp/coverage/cost.hh
  include/hpp/coverage/path.hh
  )
set(${PROJECT_NAME}_SOURCES
  src/cost.cc
  src/path.cc
  )
add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES}
                                   ${${PROJECT_NAME}_HEADERS})
target_link_libraries(${PROJECT_NAME} PUBLIC hpp-manipulation::hpp-manipulation)
install(TARGETS ${PROJECT_NAME} EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION lib)

if (CORBA)
  add_project_dependency(hpp-corbaserver)
  add_subdirectory(corba)
endif(CORBA)

set(CATKIN_PACKAGE_SHARE_DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME})

INSTALL(DIRECTORY urdf srdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
INSTALL(FILES
  package.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# The file below is necessary for xacro to find the installed xacro files
file(
  WRITE
  ${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/packages/${PROJECT_NAME}
  "")
install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/packages/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages)

# The files below are useful for using colcon
file(
  WRITE
  ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/ament_prefix_path.dsv
  "prepend-non-duplicate;AMENT_PREFIX_PATH;")
install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/ament_prefix_path.dsv
  DESTINATION share/${PROJECT_NAME}/hook)
file(WRITE
     ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/python_path.dsv
     "prepend-non-duplicate;PYTHONPATH;${PYTHON_SITELIB}")
install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/python_path.dsv
  DESTINATION share/${PROJECT_NAME}/hook)
