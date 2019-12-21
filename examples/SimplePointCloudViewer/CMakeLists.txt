
set(PCL_DIR "/usr/local/Cellar/pcl/1.9.1_1/share/pcl-1.9/")
set(VTK_DIR "/usr/local/Cellar/vtk/8.1.2_2/lib/cmake/vtk-8.1")
message("PCL_DIR...............${PCL_DIR}")
find_package(VTK REQUIRED COMPONENTS vtkCommonCore vtkRenderingFreeType vtkRenderingFreeTypeFontConfig vtkInteractionStyle vtkRenderingCore vtkRenderingOpenGL2)
find_package(PCL  REQUIRED COMPONENTS common io surface filters visualization)

find_package(Boost REQUIRED COMPONENTS system iostreams serialization)

IF(WINDOWS)
#CMAKE_POLICY(SET CMP0028 OLD)
FIND_PACKAGE(Qt5Widgets REQUIRED)
ENDIF()

add_definitions(${PCL_DEFINITIONS})

add_executable(SimplePointCloudViewer SimplePointCloudViewer.cpp PCLViewer.cpp PCLGrabber.cpp)
target_include_directories(SimplePointCloudViewer PUBLIC ${PCL_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})
target_link_libraries(SimplePointCloudViewer pointcloud ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})

IF(APPLE)
      SET_TARGET_PROPERTIES(SimplePointCloudViewer PROPERTIES LINK_FLAGS "-undefined dynamic_lookup")
ENDIF()

install(TARGETS SimplePointCloudViewer 
  RUNTIME DESTINATION bin
)

