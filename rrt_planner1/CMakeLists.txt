cmake_minimum_required(VERSION 2.8.3)
project(rrt_planner1)

add_definitions(
  -O2
  -Wall
  -g
)
add_compile_options(-std=c++11)
find_package(OMPL REQUIRED)
find_package(Boost)
include_directories(${BOOST_INCLUDE_DIRS})
link_directories(${BOOST_LIBRARY_DIRS})
add_definitions(${BOOST_DEFINITIONS})
include_directories(${OMPL_INCLUDE_DIRS})
link_directories(${OMPL_LIBRARY_DIRS})
add_definitions(${OMPL_DEFINITIONS})

find_package(catkin REQUIRED
  COMPONENTS
    costmap_2d
    geometry_msgs
    nav_core
    navfn
    nav_msgs
    pluginlib
    roscpp
    tf
    base_local_planner
)

#catkin_package(


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(rrt_lib1
  src/rrt.cpp
)
target_link_libraries(rrt_lib1 ${catkin_LIBRARIES} )

add_dependencies(rrt_lib1 ${PROJECT_NAME}_gencfg)
target_link_libraries(rrt_lib1 ${OMPL_LIBRARIES})

