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
     elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "armv7l")
      set(CMAKE_CXX_FLAGS  "-pthread -std=c++11 -fPIC -ffast-math")
      set(COMMON_INCLUDE
         /usr/include/libusb-1.0
         ./include/
         ./libs/rasp4/include/pointcloud-1.0.0/
       )
       set(PointCloud ./libs/rasp4/lib)
      message("use PointCloud SDK ./libs/rasp4/lib")
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
ELSEIF(CMAKE_SYSTEM_NAME STREQUAL "Windows")
  
  SET(WINDOWS 1)
  set(PointCloud ./libs/windows/lib)

  add_definitions(/fp:fast /W0 /bigobj /arch:AVX2 )
  ADD_DEFINITIONS(-DWINDOWS -DEnableCyAPI /wd4251  -DDLIB_HAVE_AVX2)
  set(COMMON_INCLUDE
    ${COMMON_INCLUDE}
    ./libs/windows/include/pointcloud-1.0.0/
    ${CMAKE_SOURCE_DIR}/3party/windows/libusb1.0.0/include
    Util/
    )

  if(NOT MSVC_VERSION GREATER 1800)
    SET(COMMON_LIBS setupapi strmiids strmbase Cfgmgr32 Version)
  else()
    SET(COMMON_LIBS setupapi strmiids strmbase Cfgmgr32 Version legacy_stdio_definitions)
  endif()

  
   
  
  
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

add_subdirectory(examples/DepthMeasure)
add_subdirectory(examples/ParseVXL)
add_subdirectory(examples/ExtractRawData)
add_subdirectory(examples/SimplePointCloudViewer)

