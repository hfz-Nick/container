# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_navatics_server_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED navatics_server_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(navatics_server_FOUND FALSE)
  elseif(NOT navatics_server_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(navatics_server_FOUND FALSE)
  endif()
  return()
endif()
set(_navatics_server_CONFIG_INCLUDED TRUE)

# output package information
if(NOT navatics_server_FIND_QUIETLY)
  message(STATUS "Found navatics_server: 0.0.0 (${navatics_server_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'navatics_server' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message(WARNING "${_msg}")
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(navatics_server_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_include_directories-extras.cmake")
foreach(_extra ${_extras})
  include("${navatics_server_DIR}/${_extra}")
endforeach()
