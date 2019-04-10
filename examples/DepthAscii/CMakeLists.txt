project(DepthAscii)

find_library(LIBUSB_LIBS  
  NAMES  usb 
  PATHS  /usr/local/lib  
)


add_executable(DepthAscii DepthAscii.cpp)
target_link_libraries(DepthAscii pointcloud)

             
IF(LINUX)
  set(CPACK_COMPONENTS_ALL DepthAscii)
  set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "library DepthAscii programs.")
ENDIF()
