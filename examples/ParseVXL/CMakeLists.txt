project(ParseVXL)

set(CMAKE_CXX_STANDARD 11)

find_library(LIBUSB_LIBS  
  NAMES  usb 
  PATHS  /usr/local/lib  
)

add_executable(ParseVXL ParseVXL.cpp)
target_link_libraries(ParseVXL pointcloud)
             
IF(LINUX)
  set(CPACK_COMPONENTS_ALL ParseVXL)
  set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "library ParseVXL programs.")
ENDIF()
