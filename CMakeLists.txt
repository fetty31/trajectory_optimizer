cmake_minimum_required(VERSION 3.0.2)
project(trajectory_optimizer)

set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  rospy
  std_msgs
  as_msgs
)

find_package(Eigen3 REQUIRED)
# find_package(CASADI REQUIRED)

find_path(CASADI_INCLUDE_DIR
  casadi/casadi.hpp
  HINTS ${HOME}/casadi/
)

if(CASADI_INCLUDE_DIR)
  set(CASADI_INCLUDE_DIR ${CASADI_INCLUDE_DIR} ${CASADI_INCLUDE_DIR}/casadi)
  set(CASADI_FOUND_INCLUDE TRUE)
  message(STATUS "Found CasADi include dir: ${CASADI_INCLUDE_DIR}")
else()
  message(STATUS "Could not find CasADi include dir")
endif()

if(CASADI_FOUND_INCLUDE AND CASADI_LIBRARIES)
  set(CASADI_FOUND TRUE)
endif()

find_library(CASADI_LIBRARY 
    NAMES casadi 
    PATHS ${HOME}/casadi/build/lib/
    )
if(CASADI_LIBRARY)
    set(CASADI_LIBRARIES ${CASADI_LIBRARIES} ${CASADI_LIBRARY})
endif()

if(CASADI_LIBRARIES)
  message(STATUS "Found CasADi libs: ${CASADI_LIBRARIES}")
else()
  message(STATUS "Could not find CasADi libs")
endif()

generate_dynamic_reconfigure_options(
  cfg/dynamic.cfg
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME} src/main.cpp 
                                src/TRO.cpp)

add_executable(midline src/midline.cpp
                        src/GRO.cpp)

add_executable(optimizer src/opt.cpp
                          src/SGSmooth.cpp
                          src/optimizer.cpp)

add_dependencies(${PROJECT_NAME} as_msgs_generate_messages_cpp)
                                  
add_dependencies(midline as_msgs_generate_messages_cpp)

# Dynamic reconfigure dependency
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
add_dependencies(midline ${PROJECT_NAME}_gencfg)


target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(midline ${catkin_LIBRARIES})
target_link_libraries(optimizer ${catkin_LIBRARIES} ${CASADI_LIBRARY})

