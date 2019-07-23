cmake_minimum_required (VERSION 2.8.13)
project (depth_eye)

#set(CMAKE_INCLUDE_CURRENT_DIR 1)
string(REPLACE "\n" "" SYSTEM_VERSION ${CMAKE_SYSTEM_VERSION})

string (REGEX REPLACE "^([0-9]+).*" "\\1"
  SYSTEM_MAJOR_VERSION "${SYSTEM_VERSION}")
string (REGEX REPLACE "^[0-9]+\\.([0-9]+).*" "\\1"
  SYSTEM_MINOR_VERSION "${SYSTEM_VERSION}")
string (REGEX REPLACE "^[0-9]+\\.[0-9]+\\.([0-9]+).*" "\\1"
  SYSTEM_PATCH_VERSION ${SYSTEM_VERSION})

message("CMAKE_SYSTEM_NAME = ${CMAKE_SYSTEM_NAME} VERSION=${SYSTEM_VERSION}(${SYSTEM_MAJOR_VERSION}-${SYSTEM_MINOR_VERSION}-${SYSTEM_PATCH_VERSION})") 
message("ARCH = ${CMAKE_SYSTEM_PROCESSOR}")
#set(CMAKE_SYSTEM_NAME "Linux")



if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  set(COMMON_LIBS
    /usr/local/lib
    ./lib)
     if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
      set(CMAKE_CXX_FLAGS  "-pthread -std=c++11 -fPIC -ffast-math -fpermissive")
      set(COMMON_INCLUDE
         /usr/include/libusb-1.0
         ./include/
         ./libs/ubuntu_arm/include/pointcloud-1.0.0/
       )
       set(PointCloud ./libs/ubuntu_arm/lib)
     else()
     set(CMAKE_CXX_FLAGS  "-msse2 -pthread -std=c++11 -fPIC -ffast-math")
     set(COMMON_INCLUDE
      /usr/include/libusb-1.0
      ./include/
      ./libs/macos/include/pointcloud-1.0.0/
  )
    set(PointCloud ./libs/ubuntu/lib)
    endif()
elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
  set(CMAKE_CXX_FLAGS  "-msse2 -std=c++11 -fPIC -ffast-math")
  set(COMMON_LIBS
    /usr/local/lib
    ./lib)
  set(COMMON_INCLUDE
    /usr/include/libusb-1.0
    ./include/
    ./libs/macos/include/pointcloud-1.0.0/
  )
  set(PointCloud ./libs/macos/lib)
  message("use PointCloud SDK ./libs/macos/lib")

else()
  message(FATAL_ERROR "This project not support platform:${CMAKE_SYSTEM_NAME}")
endif()

SET(BINDIR bin/)
SET(LIBDIR lib/)
SET(ARCDIR lib/)
SET(SHAREDIR share/)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${ARCDIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${LIBDIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${BINDIR})

include_directories(${COMMON_INCLUDE})
link_directories("/usr/local/lib" ${COMMON_LIBS} ${PointCloud})

#### Packaging related information -- common to both platforms
set(CPACK_PACKAGE_NAME "Depth Eye SDK")
set(CPACK_PACKAGE_VERSION "0.1")
set(CPACK_PACKAGE_CONTACT "Developer@pointcloud.ai")
set(CPACK_COMPONENTS_GROUPING ALL_COMPONENTS_IN_ONE)
set(CPACK_PACKAGE_VENDOR "pointcloud.ai")
set(CPACK_SOURCE_GENERATOR "Developer")

#add_subdirectory(examples/DepthGesture)
add_subdirectory(examples/DepthMeasure)
add_subdirectory(examples/DepthAscii)
