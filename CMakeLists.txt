cmake_minimum_required(VERSION 3.10)
project(drones_gazebo)
set (CMAKE_CXX_FLAGS "-fPIC")
find_package(gazebo REQUIRED)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
)

set(ORCA_HEADERS
  src/KdTree.h
  src/Agent.h
  src/Definitions.h
  src/Vector3.h
  )

set(ORCA_SOURCES
  src/KdTree.cpp
  src/Agent.cpp
)

add_library(DroneC src/droneclient.cpp)
add_library(DroneS src/droneserver.cpp)

add_library(ORCA ${ORCA_HEADERS} ${ORCA_SOURCES})
add_library(Collision SHARED src/collision2.cpp)
add_library(CollisionAvoidance SHARED src/CollisionAvoidance.cpp)

target_link_libraries(CollisionAvoidance DroneC DroneS ORCA) #Love this line of code so much! <3
