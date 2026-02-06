#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "traj_opt::traj_opt" for configuration "Release"
set_property(TARGET traj_opt::traj_opt APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(traj_opt::traj_opt PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libtraj_opt.so"
  IMPORTED_SONAME_RELEASE "libtraj_opt.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS traj_opt::traj_opt )
list(APPEND _IMPORT_CHECK_FILES_FOR_traj_opt::traj_opt "${_IMPORT_PREFIX}/lib/libtraj_opt.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
