#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "witmotion_ros::witmotion_ros" for configuration ""
set_property(TARGET witmotion_ros::witmotion_ros APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(witmotion_ros::witmotion_ros PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libwitmotion_ros.a"
  )

list(APPEND _cmake_import_check_targets witmotion_ros::witmotion_ros )
list(APPEND _cmake_import_check_files_for_witmotion_ros::witmotion_ros "${_IMPORT_PREFIX}/lib/libwitmotion_ros.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
