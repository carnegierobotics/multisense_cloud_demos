#
# CloudDemos/src - CMakeLists.txt
#





#
# Rules for LibMultiSenseWrapper.
#

add_library(MultiSenseWrapper SHARED
  MultiSenseWrapper.cpp)

target_link_libraries(MultiSenseWrapper
  MultiSense
  opencv_core
  opencv_calib3d
  opencv_highgui
  opencv_imgproc)


#
# Rules for LibCloudDemo
#

add_library(CloudDemo SHARED
  CloudDemo.cpp)

target_link_libraries(CloudDemo
  MultiSenseWrapper
  GL
  glut
  GLU
  pthread)



#
# Rules for TestMultiSenseWrapper
#

add_executable(TestMultiSenseWrapper
  TestMultiSenseWrapper.cpp)

target_link_libraries(TestMultiSenseWrapper
  CloudDemo
  MultiSenseWrapper)



#
# Rules for CloudDemoPointCloud
#

add_executable(CloudDemoPointCloud
  CloudDemoPointCloud.cpp)

target_link_libraries(CloudDemoPointCloud
  CloudDemo
  MultiSenseWrapper)



#
# Rules for CloudDemoKeepout.
#

add_executable(CloudDemoKeepout
  CloudDemoKeepout.cpp)

target_link_libraries(CloudDemoKeepout
  CloudDemo
  MultiSenseWrapper)


#
# Rules for CloudDemoChange
#

add_executable(CloudDemoChange
  CloudDemoChange.cpp)

target_link_libraries(CloudDemoChange
  CloudDemo
  MultiSenseWrapper)


#
# # Rules for CloudDemoVoxels
#

 add_executable(CloudDemoVoxels
   CloudDemoVoxels.cpp)

 target_link_libraries(CloudDemoVoxels
   CloudDemo
   MultiSenseWrapper)


#
# Rules for CloudDemoSimLadar
#

add_executable(CloudDemoSimLadar
  CloudDemoSimLadar.cpp)

target_link_libraries(CloudDemoSimLadar
  CloudDemo
  MultiSenseWrapper)


