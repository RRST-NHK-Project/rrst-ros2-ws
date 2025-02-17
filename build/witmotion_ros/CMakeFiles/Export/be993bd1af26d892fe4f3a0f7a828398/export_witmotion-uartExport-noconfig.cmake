#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "witmotion_ros::witmotion-uart" for configuration ""
set_property(TARGET witmotion_ros::witmotion-uart APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(witmotion_ros::witmotion-uart PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libwitmotion-uart.so"
  IMPORTED_SONAME_NOCONFIG "libwitmotion-uart.so"
  )

list(APPEND _cmake_import_check_targets witmotion_ros::witmotion-uart )
list(APPEND _cmake_import_check_files_for_witmotion_ros::witmotion-uart "${_IMPORT_PREFIX}/lib/libwitmotion-uart.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
