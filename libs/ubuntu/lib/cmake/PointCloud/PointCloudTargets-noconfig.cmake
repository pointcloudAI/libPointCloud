#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "PointCloud::pointcloud" for configuration ""
set_property(TARGET PointCloud::pointcloud APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(PointCloud::pointcloud PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libpointcloud.so"
  IMPORTED_SONAME_NOCONFIG "libpointcloud.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS PointCloud::pointcloud )
list(APPEND _IMPORT_CHECK_FILES_FOR_PointCloud::pointcloud "${_IMPORT_PREFIX}/lib/libpointcloud.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
