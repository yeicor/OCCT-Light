# Copyright (c) 2026 Capgemini Engineering Research and Development.
#
# This file is part of OCCT-Light software library.
#
# This library is free software; you can redistribute it and/or modify it under
# the terms of the GNU Affero General Public License version 3 as published
# by the Free Software Foundation, with an option to use any later version.
# Consult the file LICENSE_AGPL_30.txt included in OCCT-Light distribution
# for complete text of the license and disclaimer of any warranty.
#
# Alternatively, this file may be used under the terms of a commercial
# license or contractual agreement.
#
# SPDX-License-Identifier: AGPL-3.0-or-later

include_guard(GLOBAL)

# Set OpenCASCADE_DIR so find_package(OpenCASCADE CONFIG) finds the right install.
# Using HINTS alone is insufficient: when the OpenCASCADE config script loads all
# module targets unconditionally, a cross-module dependency ordering issue
# (Visualization targets referencing TKDE which is defined by DataExchange) causes
# OpenCASCADE_FOUND to be set to FALSE. Loading only the required modules avoids this.
if(OCCT_DIR AND NOT OpenCASCADE_DIR)
  set(OpenCASCADE_DIR "${OCCT_DIR}" CACHE PATH "OpenCASCADE CMake config dir" FORCE)
endif()

# Toolkit → OpenCASCADE module mapping.
# Each toolkit belongs to exactly one OpenCASCADE module. We load only the modules
# that contain toolkits requested by the caller, which avoids the cross-module
# dependency issue described above.
set(_OCCTL_TOOLKIT_MODULE_FoundationClasses
  TKernel TKMath)
set(_OCCTL_TOOLKIT_MODULE_ModelingData
  TKG2d TKG3d TKGeomBase TKBRep)
set(_OCCTL_TOOLKIT_MODULE_ModelingAlgorithms
  TKTopAlgo TKGeomAlgo TKShHealing TKPrim TKBool TKFeat TKFillet TKOffset
  TKMesh TKXMesh TKHLR TKExpress)
set(_OCCTL_TOOLKIT_MODULE_Visualization
  TKService TKV3d TKOpenGl TKMeshVS TKIVTK TKD3DHost TKIVtkDraw)
set(_OCCTL_TOOLKIT_MODULE_ApplicationFramework
  TKCDF TKCAF TKLCAF TKVCAF TKBin TKBinL TKBinTObj TKStd TKStdL
  TKXml TKXmlL TKXmlTObj TKTObj TKBO TKBinXCAF TKXmlXCAF TKXCAF)
set(_OCCTL_TOOLKIT_MODULE_DataExchange
  TKDE TKXSBase TKDESTEP TKDEIGES TKDESTL TKDEVRML TKRWMesh TKDECascade
  TKDEOBJ TKDEGLTF TKDEPLY)
set(_OCCTL_TOOLKIT_MODULE_Draw
  TKDraw TKTopTest TKViewerTest TKOpenGlTest TKDEDRAW TKXSDRAW TKXSDRAWDE
  TKXSDRAWObj TKXSDRAWGltf TKXSDRAWPly)

# Load OCCT configuration.
#
# OCCT 8.0.0-p1 exports all targets in a single OpenCASCADETargets.cmake (not
# per-module files).  The standard OpenCASCADEConfig.cmake tries to include
# per-module targets files and errors when they do not exist.  We bypass that
# by directly including OpenCASCADEConfig.cmake in a mode where failures from
# per-module includes are suppressed, then loading the monolithic targets file.
#
# Strategy:
#   1. Call find_package(OpenCASCADE CONFIG QUIET) — the per-module includes
#      may emit CMake errors that stop configuration.  To avoid this we wrap
#      the call in a try_include pattern.  If the standard include fails we
#      set up paths manually and load the single targets file directly.

# Pre-set OpenCASCADE_DIR so find_package finds the right install.
if(OCCT_DIR AND NOT OpenCASCADE_DIR)
  set(OpenCASCADE_DIR "${OCCT_DIR}" CACHE PATH "OpenCASCADE CMake config dir" FORCE)
endif()

# Read the version and paths from OpenCASCADEConfig.cmake without triggering
# the per-module include loop that errors on missing files.
if(EXISTS "${OpenCASCADE_DIR}/OpenCASCADEConfig.cmake")
  # Read just the non-include parts of the config file.
  set(_occt_config_file "${OpenCASCADE_DIR}/OpenCASCADEConfig.cmake")
  file(READ "${_occt_config_file}" _occt_config_content)

  # Extract version
  string(REGEX MATCH "set \\(OpenCASCADE_MAJOR_VERSION\\s+\"([^\"]+)\"\\)" _dummy "${_occt_config_content}")
  set(OpenCASCADE_MAJOR_VERSION "${CMAKE_MATCH_1}")
  string(REGEX MATCH "set \\(OpenCASCADE_MINOR_VERSION\\s+\"([^\"]+)\"\\)" _dummy "${_occt_config_content}")
  set(OpenCASCADE_MINOR_VERSION "${CMAKE_MATCH_1}")
  string(REGEX MATCH "set \\(OpenCASCADE_MAINTENANCE_VERSION\\s+\"([^\"]+)\"\\)" _dummy "${_occt_config_content}")
  set(OpenCASCADE_MAINTENANCE_VERSION "${CMAKE_MATCH_1}")

  # Compute install prefix from the config file location (same logic as OpenCASCADEConfig.cmake).
  # OpenCASCADEConfig.cmake does one LEVEL of get_filename_component to go from
  # <prefix>/lib/cmake/opencascade/OpenCASCADEConfig.cmake → <prefix>.
  # In a build-directory layout the config is at <build>/OpenCASCADEConfig.cmake,
  # so one PATH component gives <build>.
  get_filename_component(_occt_prefix "${_occt_config_file}" PATH)
  if(_occt_prefix MATCHES "/cmake$")
    get_filename_component(_occt_prefix "${_occt_prefix}" PATH)
  endif()
  if(_occt_prefix MATCHES "/lib(32|64)?$")
    get_filename_component(_occt_prefix "${_occt_prefix}" PATH)
  endif()
  if(_occt_prefix MATCHES "/share$")
    get_filename_component(_occt_prefix "${_occt_prefix}" PATH)
  endif()
  set(OpenCASCADE_INSTALL_PREFIX "${_occt_prefix}")

  # Set paths
  set(OpenCASCADE_INCLUDE_DIR "${OpenCASCADE_INSTALL_PREFIX}/include/opencascade")
  set(OpenCASCADE_LIBRARY_DIR "${OpenCASCADE_INSTALL_PREFIX}/lib")
  set(OpenCASCADE_VERSION
      "${OpenCASCADE_MAJOR_VERSION}.${OpenCASCADE_MINOR_VERSION}.${OpenCASCADE_MAINTENANCE_VERSION}")

  # Include compile definitions
  file(GLOB _occt_cdef_files
       "${OpenCASCADE_DIR}/OpenCASCADECompileDefinitionsAndFlags-*.cmake")
  foreach(_f ${_occt_cdef_files})
    include("${_f}")
  endforeach()

  # Import the single monolithic targets file
  include("${OpenCASCADE_DIR}/OpenCASCADETargets.cmake" OPTIONAL)

  # OCCT 8.0.0-p1's targets file does not set INTERFACE_INCLUDE_DIRECTORIES on
  # imported targets.  Add the include directory globally so that targets linking
  # OCCT libs can find headers like gp_Ax1.hxx, BRepGraph.hxx, etc.
  include_directories(SYSTEM "${OpenCASCADE_INCLUDE_DIR}")

  set(OpenCASCADE_FOUND TRUE)
endif()

if(NOT OpenCASCADE_FOUND)
  # Fallback: let CMake search normally
  find_package(OpenCASCADE CONFIG QUIET)
endif()

# Validate every requested toolkit target is present.  Target existence is the
# authoritative check — OpenCASCADE_FOUND can be a stale FALSE from circular
# references in the OCCT 8.0+ config even when all libraries are correctly imported.
set(_occtl_missing_toolkits "")
foreach(_toolkit IN LISTS OCCT_FIND_COMPONENTS)
  if(NOT TARGET ${_toolkit})
    list(APPEND _occtl_missing_toolkits ${_toolkit})
  endif()
endforeach()

if(_occtl_missing_toolkits)
  if(NOT OpenCASCADE_FOUND AND NOT OpenCASCADE_VERSION)
    message(FATAL_ERROR
      "OCCT-Light requires an OCCT install. Set OCCT_DIR to the directory containing "
      "OpenCASCADEConfig.cmake (typically <prefix>/lib/cmake/opencascade).\n"
      "Tried: OCCT_DIR='${OCCT_DIR}'"
    )
  endif()
  message(FATAL_ERROR
    "OCCT-Light requested OCCT toolkit(s) '${_occtl_missing_toolkits}' but they "
    "are not exported by this OCCT install at '${OpenCASCADE_INSTALL_PREFIX}'."
  )
endif()

set(OCCT_FOUND TRUE)
set(OCCT_VERSION ${OpenCASCADE_VERSION})
mark_as_advanced(OCCT_DIR)
