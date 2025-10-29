# generated from catkin/cmake/template/pkgConfig.cmake.in

# append elements to a list and remove existing duplicates from the list
# copied from catkin/cmake/list_append_deduplicate.cmake to keep pkgConfig
# self contained
macro(_list_append_deduplicate listname)
  if(NOT "${ARGN}" STREQUAL "")
    if(${listname})
      list(REMOVE_ITEM ${listname} ${ARGN})
    endif()
    list(APPEND ${listname} ${ARGN})
  endif()
endmacro()

# append elements to a list if they are not already in the list
# copied from catkin/cmake/list_append_unique.cmake to keep pkgConfig
# self contained
macro(_list_append_unique listname)
  foreach(_item ${ARGN})
    list(FIND ${listname} ${_item} _index)
    if(_index EQUAL -1)
      list(APPEND ${listname} ${_item})
    endif()
  endforeach()
endmacro()

# pack a list of libraries with optional build configuration keywords
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_pack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  set(_argn ${ARGN})
  list(LENGTH _argn _count)
  set(_index 0)
  while(${_index} LESS ${_count})
    list(GET _argn ${_index} lib)
    if("${lib}" MATCHES "^(debug|optimized|general)$")
      math(EXPR _index "${_index} + 1")
      if(${_index} EQUAL ${_count})
        message(FATAL_ERROR "_pack_libraries_with_build_configuration() the list of libraries '${ARGN}' ends with '${lib}' which is a build configuration keyword and must be followed by a library")
      endif()
      list(GET _argn ${_index} library)
      list(APPEND ${VAR} "${lib}${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}${library}")
    else()
      list(APPEND ${VAR} "${lib}")
    endif()
    math(EXPR _index "${_index} + 1")
  endwhile()
endmacro()

# unpack a list of libraries with optional build configuration keyword prefixes
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_unpack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  foreach(lib ${ARGN})
    string(REGEX REPLACE "^(debug|optimized|general)${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}(.+)$" "\\1;\\2" lib "${lib}")
    list(APPEND ${VAR} "${lib}")
  endforeach()
endmacro()


if(procedural_CONFIG_INCLUDED)
  return()
endif()
set(procedural_CONFIG_INCLUDED TRUE)

# set variables for source/devel/install prefixes
if("TRUE" STREQUAL "TRUE")
  set(procedural_SOURCE_PREFIX /home/avigne/Projets/Procedural/catkin_ws/src/Procedural)
  set(procedural_DEVEL_PREFIX /home/avigne/Projets/Procedural/catkin_ws/src/Procedural/build/devel)
  set(procedural_INSTALL_PREFIX "")
  set(procedural_PREFIX ${procedural_DEVEL_PREFIX})
else()
  set(procedural_SOURCE_PREFIX "")
  set(procedural_DEVEL_PREFIX "")
  set(procedural_INSTALL_PREFIX /usr/local)
  set(procedural_PREFIX ${procedural_INSTALL_PREFIX})
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "WARNING: package 'procedural' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message("${_msg}")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(procedural_FOUND_CATKIN_PROJECT TRUE)

if(NOT " " STREQUAL " ")
  set(procedural_INCLUDE_DIRS "")
  set(_include_dirs "")
  if(NOT " " STREQUAL " ")
    set(_report "Check the issue tracker '' and consider creating a ticket if the problem has not been reported yet.")
  elseif(NOT " " STREQUAL " ")
    set(_report "Check the website '' for information and consider reporting the problem.")
  else()
    set(_report "Report the problem to the maintainer 'avigne <avigne@todo.todo>' and request to fix the problem.")
  endif()
  foreach(idir ${_include_dirs})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif("${idir} " STREQUAL "include ")
      get_filename_component(include "${procedural_DIR}/../../../include" ABSOLUTE)
      if(NOT IS_DIRECTORY ${include})
        message(FATAL_ERROR "Project 'procedural' specifies '${idir}' as an include dir, which is not found.  It does not exist in '${include}'.  ${_report}")
      endif()
    else()
      message(FATAL_ERROR "Project 'procedural' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '/home/avigne/Projets/Procedural/catkin_ws/src/Procedural/${idir}'.  ${_report}")
    endif()
    _list_append_unique(procedural_INCLUDE_DIRS ${include})
  endforeach()
endif()

set(libraries "")
foreach(library ${libraries})
  # keep build configuration keywords, target names and absolute libraries as-is
  if("${library}" MATCHES "^(debug|optimized|general)$")
    list(APPEND procedural_LIBRARIES ${library})
  elseif(${library} MATCHES "^-l")
    list(APPEND procedural_LIBRARIES ${library})
  elseif(${library} MATCHES "^-")
    # This is a linker flag/option (like -pthread)
    # There's no standard variable for these, so create an interface library to hold it
    if(NOT procedural_NUM_DUMMY_TARGETS)
      set(procedural_NUM_DUMMY_TARGETS 0)
    endif()
    # Make sure the target name is unique
    set(interface_target_name "catkin::procedural::wrapped-linker-option${procedural_NUM_DUMMY_TARGETS}")
    while(TARGET "${interface_target_name}")
      math(EXPR procedural_NUM_DUMMY_TARGETS "${procedural_NUM_DUMMY_TARGETS}+1")
      set(interface_target_name "catkin::procedural::wrapped-linker-option${procedural_NUM_DUMMY_TARGETS}")
    endwhile()
    add_library("${interface_target_name}" INTERFACE IMPORTED)
    if("${CMAKE_VERSION}" VERSION_LESS "3.13.0")
      set_property(
        TARGET
        "${interface_target_name}"
        APPEND PROPERTY
        INTERFACE_LINK_LIBRARIES "${library}")
    else()
      target_link_options("${interface_target_name}" INTERFACE "${library}")
    endif()
    list(APPEND procedural_LIBRARIES "${interface_target_name}")
  elseif(TARGET ${library})
    list(APPEND procedural_LIBRARIES ${library})
  elseif(IS_ABSOLUTE ${library})
    list(APPEND procedural_LIBRARIES ${library})
  else()
    set(lib_path "")
    set(lib "${library}-NOTFOUND")
    # since the path where the library is found is returned we have to iterate over the paths manually
    foreach(path /home/avigne/Projets/Procedural/catkin_ws/src/Procedural/build/devel/lib;/home/avigne/Projets/supervision/catkin_ws/devel/lib;/home/avigne/Robots/Architecture/catkin_ws/install/lib;/home/avigne/Projets/Procedural/catkin_ws/devel/lib;/home/avigne/softwares/ros_noetic/devel_isolated/diff_drive_controller/lib;/home/avigne/softwares/ros_noetic/install_isolated/lib)
      find_library(lib ${library}
        PATHS ${path}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      if(lib)
        set(lib_path ${path})
        break()
      endif()
    endforeach()
    if(lib)
      _list_append_unique(procedural_LIBRARY_DIRS ${lib_path})
      list(APPEND procedural_LIBRARIES ${lib})
    else()
      # as a fall back for non-catkin libraries try to search globally
      find_library(lib ${library})
      if(NOT lib)
        message(FATAL_ERROR "Project '${PROJECT_NAME}' tried to find library '${library}'.  The library is neither a target nor built/installed properly.  Did you compile project 'procedural'?  Did you find_package() it before the subdirectory containing its code is included?")
      endif()
      list(APPEND procedural_LIBRARIES ${lib})
    endif()
  endif()
endforeach()

set(procedural_EXPORTED_TARGETS "")
# create dummy targets for exported code generation targets to make life of users easier
foreach(t ${procedural_EXPORTED_TARGETS})
  if(NOT TARGET ${t})
    add_custom_target(${t})
  endif()
endforeach()

set(depends "roscpp;rospy;std_msgs;ontologenius;mementar")
foreach(depend ${depends})
  string(REPLACE " " ";" depend_list ${depend})
  # the package name of the dependency must be kept in a unique variable so that it is not overwritten in recursive calls
  list(GET depend_list 0 procedural_dep)
  list(LENGTH depend_list count)
  if(${count} EQUAL 1)
    # simple dependencies must only be find_package()-ed once
    if(NOT ${procedural_dep}_FOUND)
      find_package(${procedural_dep} REQUIRED NO_MODULE)
    endif()
  else()
    # dependencies with components must be find_package()-ed again
    list(REMOVE_AT depend_list 0)
    find_package(${procedural_dep} REQUIRED NO_MODULE ${depend_list})
  endif()
  _list_append_unique(procedural_INCLUDE_DIRS ${${procedural_dep}_INCLUDE_DIRS})

  # merge build configuration keywords with library names to correctly deduplicate
  _pack_libraries_with_build_configuration(procedural_LIBRARIES ${procedural_LIBRARIES})
  _pack_libraries_with_build_configuration(_libraries ${${procedural_dep}_LIBRARIES})
  _list_append_deduplicate(procedural_LIBRARIES ${_libraries})
  # undo build configuration keyword merging after deduplication
  _unpack_libraries_with_build_configuration(procedural_LIBRARIES ${procedural_LIBRARIES})

  _list_append_unique(procedural_LIBRARY_DIRS ${${procedural_dep}_LIBRARY_DIRS})
  _list_append_deduplicate(procedural_EXPORTED_TARGETS ${${procedural_dep}_EXPORTED_TARGETS})
endforeach()

set(pkg_cfg_extras "")
foreach(extra ${pkg_cfg_extras})
  if(NOT IS_ABSOLUTE ${extra})
    set(extra ${procedural_DIR}/${extra})
  endif()
  include(${extra})
endforeach()
