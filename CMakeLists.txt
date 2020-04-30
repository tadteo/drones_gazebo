cmake_minimum_required(VERSION 3.10)
project(drones_gazebo)
set (CMAKE_CXX_FLAGS "-fPIC")
find_package(gazebo REQUIRED)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
)

set(ORCA_HEADERS
  src/collision_avoidance/KdTree.h
  src/collision_avoidance/Agent.h
  src/collision_avoidance/Definitions.h
  src/collision_avoidance/Vector3.h
  )

set(ORCA_SOURCES
  src/collision_avoidance/KdTree.cpp
  src/collision_avoidance/Agent.cpp
)

add_library(DroneC src/collision_avoidance/droneclient.cpp)
add_library(DroneS src/collision_avoidance/droneserver.cpp)

add_library(ORCA ${ORCA_HEADERS} ${ORCA_SOURCES})
add_library(Collision SHARED src/collision_avoidance/collision2.cpp)
add_library(CollisionAvoidance SHARED src/collision_avoidance/CollisionAvoidance.cpp)
add_library(BAPF SHARED src/collision_avoidance/BAPF.cpp)
add_library(EAPF SHARED src/collision_avoidance/EAPF.cpp)
add_library(boid SHARED src/collision_avoidance/boid.cpp)

target_link_libraries(CollisionAvoidance DroneC DroneS ORCA) #Love this line of code so much! <3
target_link_libraries(BAPF DroneC DroneS )
target_link_libraries(EAPF DroneC DroneS )
target_link_libraries(boid DroneC DroneS )