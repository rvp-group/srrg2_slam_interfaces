cmake_minimum_required(VERSION 2.8.3)
project(srrg2_slam_interfaces)

find_package(catkin REQUIRED COMPONENTS
  srrg_cmake_modules
  srrg2_core
  srrg2_solver
  srrg_hbst
)

include(${srrg_cmake_modules_INCLUDE_DIRS}/CMakeCompileOptions.txt)
#ds fight sloppyness: extend cmake compile flags to be more restrictive har har
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=pedantic")
message("WARNING CXX FLAGS SET TO ${CMAKE_CXX_FLAGS}")

#ds enable HBST code for descriptor merging and opencv support TODO move to cmake_modules?
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DSRRG_MERGE_DESCRIPTORS -DSRRG_HBST_HAS_OPENCV")

# Find Eigen3
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED) #ds can be dropped once OpenCV visualization is married with OpenGL

#srrg Create catkin package
catkin_package(
  INCLUDE_DIRS
    src
  LIBRARIES
    srrg2_slam_interfaces_library
  CATKIN_DEPENDS
    srrg2_solver
    srrg2_core
    srrg_hbst
)

#ds includes provided by this package (restrictive compile flags will be applied!)
include_directories(src)

#ds specify additional locations of header files
#ds treating them as system includes to suppress a massive wave of warnings originating from those header files (!)
include_directories(SYSTEM
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

#ds help the catkin tool on 16.04 (cmake seems unable to find single libraries, although catkin claims the link_directories call is not required)
#ds in order to avoid linking against the catkin_LIBRARIES bulk every time enable this so one can select single libraries
link_directories(${catkin_LIBRARY_DIRS})

add_subdirectory(src)
add_subdirectory(tests)
