project(DepthMeasure)

set(CMAKE_CXX_STANDARD 11)

find_library(LIBUSB_LIBS  
  NAMES  usb 
  PATHS  /usr/local/lib  
)

add_executable(DepthMeasure DepthMeasure.cpp)
target_link_libraries(DepthMeasure pointcloud)
             
IF(LINUX)
  set(CPACK_COMPONENTS_ALL DepthMeasure)
  set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "library DepthMeasure programs.")
ENDIF()
