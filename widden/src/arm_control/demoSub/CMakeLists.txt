cmake_minimum_required(VERSION 2.8.3)
project(demoSub)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(BOOST_FLAG TRUE)
if(BOOST_FLAG)
    add_definitions( -DUSE_BOOST )
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_filters
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tm_driver
  tm_kinematics
  tm_msgs
)

## System dependencies are found with CMake's conventions
LIST(APPEND CMAKE_MODULE_PATH "/usr/share/cmake-2.8/Modules")
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
find_package(Boost REQUIRED COMPONENTS system thread chrono)

file(GLOB RMLtypeII_DIR ReflexxesTypeII/src/*.cpp)
file(GLOB tm_reflexxexII_DIR src/tm_reflexxes.cpp)

link_directories(/home/peter/Desktop/tm_control/devel/lib)


# add_library(RMLTYPEII_LIB ${RMLtypeII_DIR})
# target_link_libraries(RMLTYPEII_LIB 
#     ${catkin_LIBRARIES})

# add_library(TM_REFLEXXESII_LIB ${tm_reflexxexII_DIR})
# target_link_libraries(TM_REFLEXXESII_LIB 
#     RMLTYPEII_LIB 
#     # TM_modern_driver
#     ${catkin_LIBRARIES})

# add_library(TM_OTG_LIB src/tm_otg.cpp)
# target_link_libraries(TM_OTG_LIB 
#     RMLTYPEII_LIB 
#     TM_REFLEXXESII_LIB
#     ${catkin_LIBRARIES})

###################################
## catkin specific configuration ##
###################################
if(BOOST_FLAG)
catkin_package(
    INCLUDE_DIRS include
    #  LIBRARIES holo_demo
    LIBRARIES TM_REFLEXXESII_LIB RMLTYPEII_LIB   
    CATKIN_DEPENDS apriltags_ros geometry_msgs message_filters roscpp rospy sensor_msgs std_msgs tf tm_driver tm_kinematics tm_msgs
    DEPENDS boost
    #  DEPENDS system_lib
)
else()
catkin_package(
    INCLUDE_DIRS include
    #LIBRARIES TM_REFLEXXESII_LIB RMLTYPEII_LIB   
    CATKIN_DEPENDS apriltags_ros geometry_msgs message_filters roscpp rospy sensor_msgs std_msgs tf tm_driver tm_kinematics tm_msgs 
)
endif()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  /home/peter/Desktop/tm_control/devel/include
  ReflexxesTypeII/include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
set(CMAKE_CXX_FLAGS "-std=c++11")




add_executable(tm_auo src/tm_auo.cpp)
target_link_libraries(tm_auo
  TM_REFLEXXESII_LIB
  RMLTYPEII_LIB
  TM_kinematics
  TM_print
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(handover_position_control src/handover_position_control.cpp)
target_link_libraries(handover_position_control
  TM_REFLEXXESII_LIB
  RMLTYPEII_LIB
  TM_kinematics
  TM_print
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(demo_picking src/demo_picking.cpp)
target_link_libraries(demo_picking
  TM_REFLEXXESII_LIB
  RMLTYPEII_LIB
  TM_kinematics
  TM_print
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(demo_merge src/demo_merge.cpp)
target_link_libraries(demo_merge
  TM_REFLEXXESII_LIB
  RMLTYPEII_LIB
  TM_kinematics
  TM_print
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(thesis_merge src/thesis_merge.cpp)
target_link_libraries(thesis_merge
  TM_REFLEXXESII_LIB
  RMLTYPEII_LIB
  TM_kinematics
  TM_print
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(go_certain_place src/go_certain_place.cpp)
target_link_libraries(go_certain_place
  TM_REFLEXXESII_LIB
  RMLTYPEII_LIB
  TM_kinematics
  TM_print
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)


add_executable(control_final src/control_final.cpp)
target_link_libraries(control_final
  TM_REFLEXXESII_LIB
  RMLTYPEII_LIB
  TM_kinematics
  TM_print
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)


add_executable(demo_merge_time src/demo_merge_time.cpp)
target_link_libraries(demo_merge_time
  TM_REFLEXXESII_LIB
  RMLTYPEII_LIB
  TM_kinematics
  TM_print
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(tm_demo_picking_pose src/tm_demo_picking_pose.cpp)
target_link_libraries(tm_demo_picking_pose
  TM_REFLEXXESII_LIB
  RMLTYPEII_LIB
  TM_kinematics
  TM_print
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)