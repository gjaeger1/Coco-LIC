cmake_minimum_required(VERSION 3.0.2)
project(cocolic)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -Wno-sign-compare -Wno-unused -Wno-comment -pthread")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -msse4.2")

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  eigen_conversions
  cv_bridge
  roslib
  rosbag
  tf
  message_generation
  image_transport
)

# FIND_PACKAGE(Boost REQUIRED COMPONENTS thread)
FIND_PACKAGE(Boost REQUIRED COMPONENTS filesystem iostreams program_options system serialization thread)
if(Boost_FOUND)
	INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
	LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})
endif()

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
# find_package(Sophus REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.13.0 REQUIRED)

option(ENABLE_LOOP_CLOSURE "Build the loop-closure module (requires GTSAM)" ON)
if(ENABLE_LOOP_CLOSURE)
  find_package(GTSAM REQUIRED)
  if(GTSAM_VERSION VERSION_LESS 4.1)
    message(WARNING "GTSAM ${GTSAM_VERSION} < 4.1: robust_kernel 'gnc' will be unavailable; cauchy/dcs still work.")
  endif()
  add_definitions(-DENABLE_LOOP_CLOSURE)

  # Scan Context (Kim et al.) is CC BY-NC-SA 4.0 and must not be vendored
  # into this GPLv3 tree; it is fetched into the build directory instead.
  # SCANCONTEXT_DIR may point to a pre-existing local clone for offline builds.
  if(SCANCONTEXT_DIR)
    set(scancontext_SOURCE_DIR ${SCANCONTEXT_DIR})
  else()
    include(FetchContent)
    FetchContent_Declare(scancontext
      GIT_REPOSITORY https://github.com/gisbi-kim/scancontext.git
      GIT_TAG        93672835bb85e9c03fedf0fbaf2d0491df12161e)
    FetchContent_MakeAvailable(scancontext)
  endif()
  set(SCANCONTEXT_SRC_DIR ${scancontext_SOURCE_DIR}/cpp/module/Scancontext)

  add_library(loop_lib
    src/loop/scan_context_detector.cpp
    src/loop/loop_verifier.cpp
    src/loop/pose_graph.cpp
    ${SCANCONTEXT_SRC_DIR}/Scancontext.cpp
  )
  target_include_directories(loop_lib PRIVATE ${SCANCONTEXT_SRC_DIR})
  target_link_libraries(loop_lib ${thirdparty_libraries} gtsam)
endif()

# find_package(yaml-cpp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp>=0.5)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  feature_cloud.msg
  imu_array.msg
  pose_array.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs 
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS src
  CATKIN_DEPENDS std_msgs
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  src
  ${catkin_INCLUDE_DIRS}
  ${YAML_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  # ${Sophus_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  src/camera
  src/camera/loam/include
  src/camera/tools/
  src/camera/rgb_map
)
list(APPEND thirdparty_libraries
        ${YAML_CPP_LIBRARIES}
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${CERES_LIBRARIES}
        ${PCL_LIBRARIES}
        ${Boost_LIBRARIES}
        ${Boost_FILESYSTEM_LIBRARY}
        ${Boost_SERIALIZATION_LIBRARY} # serialization
)

add_library(spline_lib src/spline/trajectory.cpp)
target_link_libraries(spline_lib ${thirdparty_libraries})

file(GLOB lidar_odometry_files
    "src/lidar/lidar_handler.cpp"
    "src/lidar/velodyne_feature_extraction.cpp",
    "src/lidar/livox_feature_extraction.cpp"
)
add_library(lidar_lib ${lidar_odometry_files})
target_link_libraries(lidar_lib spline_lib ${thirdparty_libraries})

file(GLOB r3live_files
    "src/camera/loam/*.cpp"
    "src/camera/optical_flow/*.cpp"
    "src/camera/rgb_map/*.cpp"
    "src/camera/*.cpp"
)
add_library(r3live_lib ${r3live_files})
target_link_libraries(r3live_lib ${thirdparty_libraries})

add_executable(odometry_node 
   src/odometry_node.cpp
   src/imu/imu_state_estimator.cpp
   src/imu/imu_initializer.cpp
   src/odom/msg_manager.cpp
   src/odom/trajectory_manager.cpp
   src/odom/trajectory_estimator.cpp
   src/odom/odometry_manager.cpp
   src/odom/factor/analytic_diff/marginalization_factor.cpp
   src/utils/parameter_struct.cpp
)

target_link_libraries(odometry_node 
   spline_lib 
   lidar_lib 
   r3live_lib
   ${thirdparty_libraries}
  #  fmt::fmt
)

if(ENABLE_LOOP_CLOSURE)
  target_link_libraries(odometry_node gtsam)
endif()