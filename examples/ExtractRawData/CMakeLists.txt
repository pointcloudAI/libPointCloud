project(ExtractRawData)

set(CMAKE_CXX_STANDARD 11)

LINK_DIRECTORIES(./opencv/lib)

find_library(LIBUSB_LIBS  
  NAMES  usb 
  PATHS  /usr/local/lib  
)
INCLUDE_DIRECTORIES(opencv/include)

add_executable(ExtractRawData ExtractRawData.cpp)
target_link_libraries(ExtractRawData pointcloud opencv_imgcodecs  opencv_core)
             
IF(LINUX)
  set(CPACK_COMPONENTS_ALL ExtractRawData)
  set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "library ExtractRawData programs.")
ENDIF()
