#
# Copyright (c) 2025 CNRS
# Author: Florent Lamiraux
#

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
#
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
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.

# hpp corba plugin
include(${HPP_CORBASERVER_CMAKE_PLUGIN})
include(${HPP_CORBASERVER_OMNIIDL_CMAKE})
omniidl_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/idl
                            ${HPP_CORBASERVER_DATAROOTDIR}/idl)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

set(IDL_SOURCES path)
foreach(IDL ${IDL_SOURCES})
  generate_idl_cpp(
    ${IDL}
    ${CMAKE_CURRENT_SOURCE_DIR}/idl/hpp/corbaserver/coverage ARGUMENTS
    -Wbguard_prefix=hpp_coverage_corba_idl HEADER_SUFFIX -idl.hh)
  generate_idl_python(
    ${IDL}
    ${CMAKE_CURRENT_SOURCE_DIR}/idl/hpp/corbaserver/coverage
    ENABLE_DOCSTRING
    STUBS
    hpp_stubs.corbaserver.coverage
    ARGUMENTS
    -Wbmodules=hpp_idl
    -Wbextern=common:hpp_stubs
    -Wbextern=robots:hpp_stubs.pinocchio
    -Wbextern=constraints:hpp_stubs.constraints
    -Wbextern=paths:hpp_stubs.core
    -Wbextern=steering_methods:hpp_stubs.core
    -Wbextern=path_projectors:hpp_stubs.core
    -Wbextern=path_validations:hpp_stubs.core
    -Wbextern=_constraints:hpp_stubs.core
    -Wbextern=path_planners:hpp_stubs.core
    -Wbextern=distances:hpp_stubs.core
    -Wbextern=_graph:hpp_stubs.manipulation
    -Wbextern=_path_planners:hpp_stubs.manipulation)
  install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/${IDL}-idl.hh
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hpp/corbaserver/coverage)
endforeach()

#add_custom_target(generate_idl_cpp DEPENDS ${ALL_IDL_CPP_STUBS})
add_custom_target(generate_idl_python DEPENDS ${ALL_IDL_PYTHON_STUBS})

set(PLUGIN_NAME coverage)
hpp_add_server_plugin(
  ${PLUGIN_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  SOURCES
  path.cc
  path.hh
  ${ALL_IDL_CPP_STUBS}
  path.impl.hh
  path.impl.cc
  LINK_DEPENDENCIES PUBLIC
  hpp-corbaserver::hpp-corbaserver
  ${PROJECT_NAME}
)
add_dependencies(${PLUGIN_NAME} generate_idl_python)

install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/hpp_idl/hpp/corbaserver/coverage
  DESTINATION ${PYTHON_SITELIB}/hpp_idl/hpp/corbaserver)
install(DIRECTORY
  ${CMAKE_CURRENT_BINARY_DIR}/hpp_stubs/corbaserver/coverage
  DESTINATION ${PYTHON_SITELIB}/hpp_stubs/corbaserver)
install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/hpp/corbaserver/coverage
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver)
