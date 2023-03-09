cmake_minimum_required(VERSION 2.8)
project(orb_slam3_ros)

#set(CMAKE_BUILD_TYPE Debug)
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

#SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -w")  （Problem with G2O with these）
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")
#set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wno-deprecated -O3 -march=native ")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-deprecated -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package (catkin REQUIRED COMPONENTS
roscpp
rospy
std_msgs
cv_bridge
image_transport
tf
sensor_msgs
visualization_msgs
dynamic_reconfigure
message_generation
pcl_conversions
pcl_ros
xmlrpcpp
)

find_package(OpenCV 3)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      #message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
      find_package(OpenCV 4 QUIET)
      if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
      endif()
   endif()
endif()

MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

set(LIBS_ORBSLAM
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/g2o/lib/libg2o.so
)

set(LIBS_ROS
${PROJECT_SOURCE_DIR}/orb_slam3/lib/lib${PROJECT_NAME}.so
${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/DBoW2/lib/libDBoW2.so
${OpenCV_LIBS}
${PCL_LIBS}
${catkin_LIBRARIES}
)


add_message_files(
  FILES
  KF.msg
  KFs.msg
  CvKeyPoint.msg
  Descriptor.msg
  MP.msg
  KFred.msg
  MPred.msg
  Map.msg
  PreintegratedIMU.msg
  RefPosKF.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package (
  CATKIN_DEPENDS roscpp rospy std_msgs cv_bridge image_transport tf sensor_msgs visualization_msgs dynamic_reconfigure message_runtime
  pcl_conversions pcl_ros
  LIBRARIES {PROJECT_NAME} libDBoW2 libg2o
)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/orb_slam3
${PROJECT_SOURCE_DIR}/orb_slam3/include
${PROJECT_SOURCE_DIR}/ros/include
${PROJECT_SOURCE_DIR}/ros/include/CameraModels
${EIGEN3_INCLUDE_DIR}
${catkin_INCLUDE_DIRS}
${Pangolin_INCLUDE_DIRS}
)

include(${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/DBoW2/CMakeLists.txt)
include(${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/g2o/CMakeLists.txt)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/orb_slam3/lib)


add_library(${PROJECT_NAME} SHARED
#orb_slam3/src/System.cc
orb_slam3/src/ClientSystem.cc
orb_slam3/src/ServerSystem.cc
orb_slam3/src/CentralControl.cc
orb_slam3/src/Communicator.cc
orb_slam3/src/ClientHandler.cc
orb_slam3/src/Tracking.cc
orb_slam3/src/LocalMapping.cc
orb_slam3/src/LoopClosing.cc
orb_slam3/src/ORBextractor.cc
orb_slam3/src/ORBmatcher.cc
orb_slam3/src/FrameDrawer.cc
orb_slam3/src/ClientViewer.cc
orb_slam3/src/Converter.cc
orb_slam3/src/MapPoint.cc
orb_slam3/src/KeyFrame.cc
orb_slam3/src/Atlas.cc
orb_slam3/src/Map.cc
orb_slam3/src/MapDrawer.cc
orb_slam3/src/Optimizer.cc
orb_slam3/src/PnPsolver.cc
orb_slam3/src/Frame.cc
orb_slam3/src/KeyFrameDatabase.cc
orb_slam3/src/Sim3Solver.cc
orb_slam3/src/Initializer.cc
orb_slam3/src/ServerViewer.cc
orb_slam3/src/ImuTypes.cc
orb_slam3/src/G2oTypes.cc
orb_slam3/src/CameraModels/Pinhole.cpp
orb_slam3/src/CameraModels/KannalaBrandt8.cpp
orb_slam3/src/OptimizableTypes.cpp
orb_slam3/src/MLPnPsolver.cpp
#orb_slam3/include/System.h
orb_slam3/include/FrameDrawer.h
orb_slam3/include/ClientViewer.h
orb_slam3/include/ClientSystem.h
orb_slam3/include/ServerSystem.h
orb_slam3/include/CentralControl.h
orb_slam3/include/Communicator.h
orb_slam3/include/ClientHandler.h
orb_slam3/include/Tracking.h
orb_slam3/include/LocalMapping.h
orb_slam3/include/LoopClosing.h
orb_slam3/include/ORBextractor.h
orb_slam3/include/ORBmatcher.h
orb_slam3/include/Datatypes.h
orb_slam3/include/Converter.h
orb_slam3/include/MapPoint.h
orb_slam3/include/KeyFrame.h
orb_slam3/include/Atlas.h
orb_slam3/include/Map.h
orb_slam3/include/MapDrawer.h
orb_slam3/include/Optimizer.h
orb_slam3/include/PnPsolver.h
orb_slam3/include/Frame.h
orb_slam3/include/KeyFrameDatabase.h
orb_slam3/include/Sim3Solver.h
orb_slam3/include/Initializer.h
orb_slam3/include/ServerViewer.h
orb_slam3/include/ImuTypes.h
orb_slam3/include/G2oTypes.h
orb_slam3/include/CameraModels/GeometricCamera.h
orb_slam3/include/CameraModels/Pinhole.h
orb_slam3/include/CameraModels/KannalaBrandt8.h
orb_slam3/include/OptimizableTypes.h
orb_slam3/include/MLPnPsolver.h
orb_slam3/include/TwoViewReconstruction.h
orb_slam3/src/TwoViewReconstruction.cc)

add_dependencies (${PROJECT_NAME} g2o DBoW2)
#add_subdirectory(${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/g2o)

target_link_libraries(${PROJECT_NAME} ${LIBS_ORBSLAM}
  -lboost_serialization
  -lcrypto
)

add_executable (${PROJECT_NAME}_client
  ros/src/ClientNode.cc
 )
add_dependencies (${PROJECT_NAME}_client ${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}_client
 ${LIBS_ROS}
 )

add_executable (${PROJECT_NAME}_mono_inertial
   ros/src/MonoInertialNode.cc
)
add_dependencies (${PROJECT_NAME}_mono_inertial ${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}_mono_inertial
  ${LIBS_ROS}
)

add_executable (${PROJECT_NAME}_rgbd
   ros/src/RGBDNode.cc
)
add_dependencies (${PROJECT_NAME}_rgbd ${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}_rgbd
  ${LIBS_ROS}
)

add_executable (${PROJECT_NAME}_server
  ros/src/ServerNode.cc
 )

add_dependencies (${PROJECT_NAME}_server ${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}_server ${LIBS_ROS})


install(TARGETS ${PROJECT_NAME}_client ${PROJECT_NAME}_server
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME} DBoW2 g2o
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

install(DIRECTORY ros/config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/config
)

install(DIRECTORY orb_slam3/config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/orb_slam2/config
)

install(DIRECTORY orb_slam3/Vocabulary/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/orb_slam2/Vocabulary
)
